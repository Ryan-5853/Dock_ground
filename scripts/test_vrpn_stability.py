#!/usr/bin/env python3
"""Capture and analyze a VRPN tracker stream without the flight controller."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import queue
import re
import socket
import subprocess
import sys
import threading
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "config" / "ground.gs-link.json"
DEFAULT_PROBE = REPO_ROOT / "build" / "dock_ground_vrpn_probe"


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    index = round(fraction * (len(ordered) - 1))
    return ordered[max(0, min(index, len(ordered) - 1))]


def json_number(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return value


def load_config(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exception:
        raise SystemExit(f"cannot read config {path}: {exception}") from exception
    if not isinstance(data, dict):
        raise SystemExit(f"config root must be an object: {path}")
    return data


def config_defaults(
    config: dict[str, Any],
) -> tuple[str, int, float, float, float]:
    vrpn = config.get("vrpn", {})
    control = config.get("control", {})
    if not isinstance(vrpn, dict) or not isinstance(control, dict):
        raise SystemExit("config vrpn/control sections must be objects")
    endpoint = str(vrpn.get("endpoint", "MCAvatar@192.168.31.221:3883"))
    configured_sensor = vrpn.get("sensor_id", 1)
    sensor = -1 if configured_sensor is None else int(configured_sensor)
    expected_rate = float(vrpn.get("expected_rate_hz", 120.0))
    position_scale = float(vrpn.get("position_scale", 0.001))
    max_pose_age_ms = float(control.get("max_pose_age_ms", 50.0))
    return endpoint, sensor, expected_rate, position_scale, max_pose_age_ms


def stream_probe(
    command: list[str],
    log_path: Path,
    timeout_s: float,
) -> int:
    print("running:", " ".join(command))
    try:
        process = subprocess.Popen(
            command,
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except OSError as exception:
        raise SystemExit(f"cannot start VRPN probe: {exception}") from exception

    assert process.stdout is not None
    output_queue: queue.Queue[str | None] = queue.Queue()

    def read_output() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            output_queue.put(line)
        output_queue.put(None)

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    deadline = time.monotonic() + timeout_s
    try:
        with log_path.open("w", encoding="utf-8") as log:
            finished = False
            while time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                try:
                    line = output_queue.get(timeout=min(0.5, remaining))
                except queue.Empty:
                    continue
                if line is None:
                    finished = True
                    break
                log.write(line)
                log.flush()
                print(line, end="")
            if not finished:
                process.kill()
                process.wait()
                print(
                    f"VRPN probe exceeded timeout of {timeout_s:.1f}s",
                    file=sys.stderr,
                )
                return 124
        return process.wait(timeout=5)
    finally:
        reader.join(timeout=1)
        if process.poll() is None:
            process.kill()
            process.wait()


def endpoint_host(endpoint: str) -> str:
    device_host = endpoint.split("@", 1)[-1]
    if device_host.startswith("["):
        closing = device_host.find("]")
        if closing > 0:
            return device_host[1:closing]
    return device_host.rsplit(":", 1)[0]


def run_network_ping(
    endpoint: str,
    output_dir: Path,
    count: int = 20,
) -> dict[str, Any]:
    host = endpoint_host(endpoint)
    command = (
        ["ping", "-n", str(count), host]
        if os.name == "nt"
        else ["ping", "-c", str(count), host]
    )
    path = output_dir / "ping.log"
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=count + 15,
            check=False,
        )
        combined = (completed.stdout or "") + (completed.stderr or "")
        path.write_text(combined, encoding="utf-8")
        match = re.search(r"(\d+(?:\.\d+)?)%\s*(?:packet\s+)?loss", combined, re.I)
        return {
            "host": host,
            "command": command,
            "exit_code": completed.returncode,
            "packet_loss_percent": float(match.group(1)) if match else None,
            "log": str(path),
        }
    except (OSError, subprocess.TimeoutExpired) as exception:
        path.write_text(f"{exception}\n", encoding="utf-8")
        return {
            "host": host,
            "command": command,
            "exit_code": None,
            "error": str(exception),
            "log": str(path),
        }


def quaternion_angle_deg(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float | None:
    norm_product = math.sqrt(sum(value * value for value in first)) * math.sqrt(
        sum(value * value for value in second)
    )
    if norm_product < 1e-12:
        return None
    dot = sum(a * b for a, b in zip(first, second)) / norm_product
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def analyze_csv(
    path: Path,
    *,
    expected_rate_hz: float,
    position_scale: float,
    gap_warning_ms: float,
    gap_fail_ms: float,
    max_pose_age_ms: float,
    min_rate_ratio: float,
    static_test: bool,
    max_position_step_m: float,
    max_rotation_step_deg: float,
) -> dict[str, Any]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise SystemExit(f"VRPN CSV contains no samples: {path}")

    received = [
        int(row["received_mono_ns"])
        for row in rows
        if row.get("received_mono_ns")
    ]
    if len(received) != len(rows):
        raise SystemExit(f"VRPN CSV contains malformed receive timestamps: {path}")

    arrival_intervals: list[float] = []
    source_intervals: list[float] = []
    source_clock_deltas: list[float] = []
    qnorm_errors: list[float] = []
    position_steps_m: list[float] = []
    rotation_steps_deg: list[float] = []
    nonfinite_rows = 0
    missing_source_time = 0
    source_regressions = 0
    duplicate_source_times = 0
    callback_bursts = 0
    sensors: Counter[int] = Counter()
    source_times_by_sensor: dict[int, set[int]] = {}
    previous_source_by_sensor: dict[int, int] = {}
    previous_position_by_sensor: dict[int, tuple[float, float, float]] = {}
    previous_quaternion_by_sensor: dict[
        int, tuple[float, float, float, float]
    ] = {}
    gap_events: list[dict[str, Any]] = []

    for row in rows:
        sensor_id = int(row["sensor"])
        sensors[sensor_id] += 1
        arrival = finite_float(row.get("arrival_interval_ms"))
        if arrival is not None and arrival >= 0.0:
            arrival_intervals.append(arrival)
            if arrival < 1.0:
                callback_bursts += 1
            if arrival > gap_warning_ms:
                gap_events.append(
                    {
                        "received_mono_ns": int(row["received_mono_ns"]),
                        "wall_time_ns": int(row["wall_time_ns"]),
                        "gap_ms": round(arrival, 6),
                        "source_time_interval_ms": json_number(
                            finite_float(row.get("source_time_interval_ms"))
                        ),
                        "raw_x": json_number(finite_float(row.get("raw_x"))),
                        "raw_y": json_number(finite_float(row.get("raw_y"))),
                        "raw_z": json_number(finite_float(row.get("raw_z"))),
                    }
                )

        source_present = row.get("source_time_present") == "1"
        source_time = int(row.get("source_time_ns") or 0)
        if not source_present or source_time <= 0:
            missing_source_time += 1
        else:
            source_times = source_times_by_sensor.setdefault(sensor_id, set())
            previous_source_time = previous_source_by_sensor.get(sensor_id)
            if source_time in source_times:
                duplicate_source_times += 1
            if previous_source_time is not None:
                if source_time < previous_source_time:
                    source_regressions += 1
                elif source_time > previous_source_time:
                    source_intervals.append(
                        (source_time - previous_source_time) / 1_000_000.0
                    )
            previous_source_by_sensor[sensor_id] = source_time
            source_times.add(source_time)

        clock_delta = finite_float(row.get("source_clock_delta_ms"))
        if clock_delta is not None:
            source_clock_deltas.append(clock_delta)

        raw_position = (
            finite_float(row.get("raw_x")),
            finite_float(row.get("raw_y")),
            finite_float(row.get("raw_z")),
        )
        quaternion = (
            finite_float(row.get("qx")),
            finite_float(row.get("qy")),
            finite_float(row.get("qz")),
            finite_float(row.get("qw")),
        )
        qnorm = finite_float(row.get("qnorm"))
        if any(value is None for value in (*raw_position, *quaternion, qnorm)):
            nonfinite_rows += 1
            previous_position_by_sensor.pop(sensor_id, None)
            previous_quaternion_by_sensor.pop(sensor_id, None)
            continue

        position = (float(raw_position[0]), float(raw_position[1]), float(raw_position[2]))
        orientation = (
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        )
        qnorm_errors.append(abs(float(qnorm) - 1.0))
        previous_position = previous_position_by_sensor.get(sensor_id)
        previous_quaternion = previous_quaternion_by_sensor.get(sensor_id)
        if previous_position is not None:
            delta = tuple(a - b for a, b in zip(position, previous_position))
            position_steps_m.append(
                math.sqrt(sum(value * value for value in delta))
                * position_scale
            )
        if previous_quaternion is not None:
            angle = quaternion_angle_deg(previous_quaternion, orientation)
            if angle is not None:
                rotation_steps_deg.append(angle)
        previous_position_by_sensor[sensor_id] = position
        previous_quaternion_by_sensor[sensor_id] = orientation

    duration_s = (max(received) - min(received)) / 1_000_000_000.0
    effective_duration_s = max(duration_s, 1.0 / max(1.0, expected_rate_hz))
    measured_rate_hz = len(rows) / effective_duration_s
    gap_over_warning = len(gap_events)
    gap_over_fail = sum(
        event["gap_ms"] >= gap_fail_ms for event in gap_events
    )
    max_gap_ms = max(arrival_intervals, default=0.0)
    p99_gap_ms = percentile(arrival_intervals, 0.99)
    max_clock_drift_ms = (
        max(source_clock_deltas) - min(source_clock_deltas)
        if source_clock_deltas
        else None
    )
    failures: list[str] = []
    warnings: list[str] = []

    minimum_rate_hz = expected_rate_hz * min_rate_ratio
    if measured_rate_hz < minimum_rate_hz:
        failures.append(
            f"sample rate {measured_rate_hz:.2f} Hz is below "
            f"{minimum_rate_hz:.2f} Hz"
        )
    if max_gap_ms > gap_fail_ms:
        failures.append(
            f"maximum callback gap {max_gap_ms:.2f} ms exceeds "
            f"{gap_fail_ms:.2f} ms"
        )
    if nonfinite_rows:
        failures.append(f"{nonfinite_rows} rows contain non-finite pose data")
    if qnorm_errors and max(qnorm_errors) > 1e-3:
        failures.append(
            f"maximum quaternion norm error {max(qnorm_errors):.6g} "
            "exceeds 1e-3"
        )
    if source_regressions:
        failures.append(f"{source_regressions} source timestamps moved backwards")
    if duplicate_source_times:
        failures.append(f"{duplicate_source_times} duplicate source timestamps")
    if static_test and position_steps_m:
        maximum_step = max(position_steps_m)
        if maximum_step > max_position_step_m:
            failures.append(
                f"static position step {maximum_step:.4f} m exceeds "
                f"{max_position_step_m:.4f} m"
            )
    if static_test and rotation_steps_deg:
        maximum_rotation = max(rotation_steps_deg)
        if maximum_rotation > max_rotation_step_deg:
            failures.append(
                f"static rotation step {maximum_rotation:.2f} deg exceeds "
                f"{max_rotation_step_deg:.2f} deg"
            )

    if p99_gap_ms is not None and p99_gap_ms > max_pose_age_ms:
        warnings.append(
            f"p99 callback gap {p99_gap_ms:.2f} ms exceeds the ground station "
            f"max_pose_age_ms={max_pose_age_ms:.2f} ms"
        )
    if max_gap_ms > max_pose_age_ms:
        warnings.append(
            f"maximum callback gap {max_gap_ms:.2f} ms exceeds the ground "
            f"station max_pose_age_ms={max_pose_age_ms:.2f} ms and can force "
            "a handoff reset"
        )
    if gap_over_warning:
        warnings.append(
            f"{gap_over_warning} callback gaps exceed "
            f"{gap_warning_ms:.2f} ms"
        )
    if callback_bursts:
        warnings.append(
            f"{callback_bursts} callbacks arrived less than 1 ms after the "
            "previous callback, indicating catch-up bursts"
        )
    if missing_source_time:
        warnings.append(f"{missing_source_time} samples have no source timestamp")
    if max_clock_drift_ms is not None and max_clock_drift_ms > 5.0:
        warnings.append(
            f"source clock delta varies by {max_clock_drift_ms:.2f} ms during "
            "the capture"
        )

    verdict = "FAIL" if failures else ("WARN" if warnings else "PASS")
    return {
        "schema": 1,
        "verdict": verdict,
        "samples": len(rows),
        "sensors": {str(key): value for key, value in sorted(sensors.items())},
        "duration_s": duration_s,
        "measured_rate_hz": measured_rate_hz,
        "expected_rate_hz": expected_rate_hz,
        "minimum_expected_rate_hz": minimum_rate_hz,
        "arrival_gap_ms": {
            "p50": percentile(arrival_intervals, 0.50),
            "p95": percentile(arrival_intervals, 0.95),
            "p99": p99_gap_ms,
            "max": max_gap_ms,
            "over_warning": gap_over_warning,
            "over_fail": gap_over_fail,
            "under_1ms_bursts": callback_bursts,
        },
        "source_time": {
            "missing": missing_source_time,
            "regressions": source_regressions,
            "duplicates": duplicate_source_times,
            "interval_p99_ms": percentile(source_intervals, 0.99),
            "clock_delta_first_ms": (
                source_clock_deltas[0] if source_clock_deltas else None
            ),
            "clock_delta_last_ms": (
                source_clock_deltas[-1] if source_clock_deltas else None
            ),
            "clock_delta_range_ms": max_clock_drift_ms,
        },
        "pose_quality": {
            "nonfinite_rows": nonfinite_rows,
            "max_quaternion_norm_error": (
                max(qnorm_errors) if qnorm_errors else None
            ),
            "max_position_step_m": (
                max(position_steps_m) if position_steps_m else None
            ),
            "max_rotation_step_deg": (
                max(rotation_steps_deg) if rotation_steps_deg else None
            ),
        },
        "thresholds": {
            "gap_warning_ms": gap_warning_ms,
            "gap_fail_ms": gap_fail_ms,
            "max_pose_age_ms": max_pose_age_ms,
            "min_rate_ratio": min_rate_ratio,
            "static_test": static_test,
            "max_position_step_m": max_position_step_m,
            "max_rotation_step_deg": max_rotation_step_deg,
        },
        "failures": failures,
        "warnings": warnings,
        "largest_gap_events": sorted(
            gap_events, key=lambda event: event["gap_ms"], reverse=True
        )[:20],
    }


def write_summary(
    result: dict[str, Any],
    output_dir: Path,
    metadata: dict[str, Any],
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    document = {"metadata": metadata, "analysis": result}
    json_path = output_dir / "summary.json"
    text_path = output_dir / "summary.txt"
    with json_path.open("w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")

    arrival = result["arrival_gap_ms"]
    source = result["source_time"]
    quality = result["pose_quality"]

    def format_value(value: Any, digits: int = 3) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, float):
            return f"{value:.{digits}f}"
        return str(value)

    lines = [
        f"VERDICT: {result['verdict']}",
        f"endpoint: {metadata.get('endpoint')}",
        f"sensor: {metadata.get('sensor')}",
        f"samples: {result['samples']}",
        f"measured rate: {result['measured_rate_hz']:.3f} Hz "
        f"(expected {result['expected_rate_hz']:.3f} Hz)",
        f"gap p50/p95/p99/max: "
        f"{format_value(arrival['p50'])} / "
        f"{format_value(arrival['p95'])} / "
        f"{format_value(arrival['p99'])} / "
        f"{format_value(arrival['max'])} ms",
        f"gap warnings/failures: {arrival['over_warning']} / "
        f"{arrival['over_fail']}",
        f"sub-1ms catch-up callbacks: {arrival['under_1ms_bursts']}",
        f"source timestamp missing/duplicate/regressions: "
        f"{source['missing']} / {source['duplicates']} / "
        f"{source['regressions']}",
        f"source clock delta range: "
        f"{format_value(source['clock_delta_range_ms'])} ms",
        f"max quaternion norm error: "
        f"{format_value(quality['max_quaternion_norm_error'], 6)}",
        f"max static position step: "
        f"{format_value(quality['max_position_step_m'], 4)} m",
        f"max static rotation step: "
        f"{format_value(quality['max_rotation_step_deg'], 3)} deg",
    ]
    if result["failures"]:
        lines.append("failures:")
        lines.extend(f"  - {item}" for item in result["failures"])
    if result["warnings"]:
        lines.append("warnings:")
        lines.extend(f"  - {item}" for item in result["warnings"])
    text_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, text_path


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture every VRPN tracker callback and assess rate, gap, "
            "timestamp, and static-pose stability."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--endpoint")
    parser.add_argument("--sensor", type=int)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--expected-rate-hz", type=float)
    parser.add_argument("--gap-warning-ms", type=float, default=25.0)
    parser.add_argument("--gap-fail-ms", type=float, default=100.0)
    parser.add_argument("--min-rate-ratio", type=float, default=0.80)
    parser.add_argument("--max-position-step-m", type=float, default=0.05)
    parser.add_argument("--max-rotation-step-deg", type=float, default=20.0)
    parser.add_argument(
        "--motion",
        action="store_true",
        help="Do not fail on expected rigid-body movement.",
    )
    parser.add_argument("--probe", type=Path, default=DEFAULT_PROBE)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--label", default="static")
    parser.add_argument(
        "--analyze-only",
        type=Path,
        help="Analyze an existing vrpn_samples.csv without touching VRPN.",
    )
    parser.add_argument(
        "--no-strict",
        action="store_true",
        help="Always exit zero after writing the report.",
    )
    parser.add_argument(
        "--skip-ping",
        action="store_true",
        help="Do not run the best-effort ICMP network probe.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    config = load_config(args.config)
    (
        config_endpoint,
        config_sensor,
        config_expected_rate,
        config_position_scale,
        config_max_pose_age,
    ) = config_defaults(config)
    endpoint = args.endpoint or config_endpoint
    sensor = config_sensor if args.sensor is None else args.sensor
    expected_rate = args.expected_rate_hz or config_expected_rate
    output_dir = (
        args.output_dir
        or REPO_ROOT
        / "logs"
        / "vrpn-stability"
        / f"run_{utc_timestamp()}_{args.label}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.analyze_only or output_dir / "vrpn_samples.csv"
    started_utc = datetime.now(timezone.utc).isoformat()
    try:
        load_average: list[float] | None = list(os.getloadavg())
    except (AttributeError, OSError):
        load_average = None
    metadata = {
        "schema": 1,
        "started_utc": started_utc,
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "load_average": load_average,
        "cwd": os.getcwd(),
        "config": str(args.config.resolve()),
        "endpoint": endpoint,
        "sensor": sensor,
        "duration_requested_s": args.duration,
        "expected_rate_hz": expected_rate,
        "position_scale": config_position_scale,
        "max_pose_age_ms": config_max_pose_age,
        "auto_origin": config.get("vrpn", {}).get("auto_origin"),
        "trust_source_timestamp": config.get("vrpn", {}).get(
            "trust_source_timestamp"
        ),
        "axes_from_vrpn": config.get("vrpn", {}).get("axes_from_vrpn"),
        "static_test": not args.motion,
        "probe": str(args.probe.resolve()),
    }

    probe_exit_code: int | None = None
    if args.analyze_only is None and not args.skip_ping:
        metadata["ping"] = run_network_ping(endpoint, output_dir)
    if args.analyze_only is None:
        if not args.probe.is_file():
            raise SystemExit(
                f"VRPN probe is missing: {args.probe}\n"
                f"Build it with: cmake --build {REPO_ROOT / 'build'} "
                "--target dock_ground_vrpn_probe"
            )
        command = [
            str(args.probe.resolve()),
            endpoint,
            "--duration",
            str(args.duration),
            "--sensor",
            str(sensor),
            "--expected-rate-hz",
            str(expected_rate),
            "--gap-warning-ms",
            str(args.gap_warning_ms),
            "--csv",
            str(csv_path),
        ]
        probe_exit_code = stream_probe(
            command,
            output_dir / "probe.log",
            timeout_s=args.duration + 30.0,
        )
        metadata["probe_exit_code"] = probe_exit_code

    if not csv_path.is_file():
        raise SystemExit(f"VRPN sample CSV is missing: {csv_path}")
    result = analyze_csv(
        csv_path,
        expected_rate_hz=expected_rate,
        position_scale=config_position_scale,
        gap_warning_ms=args.gap_warning_ms,
        gap_fail_ms=args.gap_fail_ms,
        max_pose_age_ms=config_max_pose_age,
        min_rate_ratio=args.min_rate_ratio,
        static_test=not args.motion,
        max_position_step_m=args.max_position_step_m,
        max_rotation_step_deg=args.max_rotation_step_deg,
    )
    if probe_exit_code not in (None, 0):
        result["warnings"].append(
            f"VRPN probe exited with code {probe_exit_code}"
        )
        if result["verdict"] == "PASS":
            result["verdict"] = "WARN"
    packet_loss = (
        metadata.get("ping", {}).get("packet_loss_percent")
        if isinstance(metadata.get("ping"), dict)
        else None
    )
    if packet_loss is not None and packet_loss > 0:
        result["warnings"].append(
            f"ICMP reported {packet_loss:.2f}% packet loss; interpret only as "
            "supporting evidence because ICMP priority may differ from VRPN"
        )
        if result["verdict"] == "PASS":
            result["verdict"] = "WARN"
    metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
    json_path, text_path = write_summary(result, output_dir, metadata)

    print()
    print(text_path.read_text(encoding="utf-8"), end="")
    print(f"json: {json_path}")
    print(f"csv:  {csv_path}")
    if result["verdict"] == "FAIL" and not args.no_strict:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
