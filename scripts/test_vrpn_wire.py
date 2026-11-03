#!/usr/bin/env python3
"""Run VRPN stability capture with synchronized packet capture and ping."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "config" / "ground.gs-link.json"
DEFAULT_STABILITY = REPO_ROOT / "scripts" / "test_vrpn_stability.py"
DEFAULT_PROBE = REPO_ROOT / "build" / "dock_ground_vrpn_probe"


def utc_label() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def load_config(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exception:
        raise SystemExit(f"cannot read config {path}: {exception}") from exception
    if not isinstance(data, dict):
        raise SystemExit(f"config root must be an object: {path}")
    return data


def endpoint_parts(endpoint: str) -> tuple[str, str, int]:
    sender, separator, address = endpoint.partition("@")
    if not separator or not sender or not address:
        raise SystemExit(f"invalid VRPN endpoint: {endpoint}")
    if address.startswith("["):
        closing = address.find("]")
        if closing < 0 or closing + 2 > len(address) or address[closing + 1] != ":":
            raise SystemExit(f"invalid bracketed VRPN endpoint: {endpoint}")
        host = address[1:closing]
        port_text = address[closing + 2 :]
    else:
        host, colon, port_text = address.rpartition(":")
        if not colon:
            raise SystemExit(f"VRPN endpoint has no UDP/TCP port: {endpoint}")
    try:
        port = int(port_text)
    except ValueError as exception:
        raise SystemExit(f"invalid VRPN port in endpoint: {endpoint}") from exception
    if not 1 <= port <= 65535:
        raise SystemExit(f"VRPN port outside [1, 65535]: {endpoint}")
    return sender, host, port


def detect_interface(host: str) -> str:
    try:
        completed = subprocess.run(
            ["ip", "route", "get", host],
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exception:
        raise SystemExit(
            "cannot auto-detect interface; pass --interface. "
            f"ip route failed: {exception}"
        ) from exception
    match = re.search(r"\bdev\s+(\S+)", completed.stdout)
    if completed.returncode != 0 or match is None:
        raise SystemExit(
            "cannot auto-detect interface; pass --interface. "
            f"ip route output: {(completed.stdout + completed.stderr).strip()}"
        )
    return match.group(1)


def parse_ping_output(path: Path) -> dict[str, Any]:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return {}
    loss = re.search(r"(\d+(?:\.\d+)?)%\s*(?:packet\s+)?loss", text, re.I)
    rtt = re.search(
        r"(?:rtt|round-trip).*?=\s*"
        r"([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+)",
        text,
        re.I,
    )
    result: dict[str, Any] = {
        "packet_loss_percent": float(loss.group(1)) if loss else None,
    }
    if rtt:
        result.update(
            {
                "rtt_min_ms": float(rtt.group(1)),
                "rtt_avg_ms": float(rtt.group(2)),
                "rtt_max_ms": float(rtt.group(3)),
                "rtt_mdev_ms": float(rtt.group(4)),
            }
        )
    return result


def parse_endpoint_token(token: str) -> tuple[str, int] | None:
    if token.startswith("["):
        closing = token.find("]")
        if closing < 0 or closing + 2 > len(token) or token[closing + 1] != ".":
            return None
        host = token[1:closing]
        port_text = token[closing + 2 :]
    else:
        host, separator, port_text = token.rpartition(".")
        if not separator:
            return None
    if not port_text.isdigit():
        return None
    return host, int(port_text)


PACKET_PATTERN = re.compile(
    r"^(?P<timestamp>\d+\.\d+)\s+IP6?\s+"
    r"(?P<source>\S+)\s+>\s+(?P<destination>\S+):\s+"
    r"UDP,\s+length\s+(?P<length>\d+)"
)


def timestamp_ns(value: str) -> int:
    seconds, dot, fraction = value.partition(".")
    if not dot:
        return int(seconds) * 1_000_000_000
    nanos = (fraction + "000000000")[:9]
    return int(seconds) * 1_000_000_000 + int(nanos)


def parse_packet_dump(
    text: str,
    target_host: str,
    *,
    expected_callback_count: int | None = None,
    callback_start_ns: int | None = None,
    callback_end_ns: int | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    packets: list[dict[str, Any]] = []
    for line in text.splitlines():
        match = PACKET_PATTERN.match(line)
        if match is None:
            continue
        source = parse_endpoint_token(match.group("source"))
        destination = parse_endpoint_token(match.group("destination"))
        if source is None or destination is None:
            continue
        source_host, source_port = source
        destination_host, destination_port = destination
        packets.append(
            {
                "timestamp_ns": timestamp_ns(match.group("timestamp")),
                "source_host": source_host,
                "source_port": source_port,
                "destination_host": destination_host,
                "destination_port": destination_port,
                "length": int(match.group("length")),
            }
        )

    counts: Counter[tuple[str, int, str, int]] = Counter()
    for packet in packets:
        counts[
            (
                packet["source_host"],
                packet["source_port"],
                packet["destination_host"],
                packet["destination_port"],
            )
        ] += 1
    flows = [
        {
            "source_host": key[0],
            "source_port": key[1],
            "destination_host": key[2],
            "destination_port": key[3],
            "packets": count,
        }
        for key, count in counts.most_common()
    ]
    from_target = [
        flow for flow in flows if flow["source_host"] == target_host
    ]
    candidates = from_target or flows
    if candidates:
        def flow_score(flow: dict[str, Any]) -> tuple[float, float, float]:
            flow_packets = [
                packet["timestamp_ns"]
                for packet in packets
                if (
                    packet["source_host"],
                    packet["source_port"],
                    packet["destination_host"],
                    packet["destination_port"],
                )
                == (
                    flow["source_host"],
                    flow["source_port"],
                    flow["destination_host"],
                    flow["destination_port"],
                )
            ]
            if not flow_packets:
                return (math.inf, math.inf, math.inf)
            flow_start = min(flow_packets)
            flow_end = max(flow_packets)
            count_error = 0.0
            duration_error = 0.0
            boundary_error = 0.0
            if expected_callback_count is not None:
                if callback_start_ns is not None and callback_end_ns is not None:
                    within_count = sum(
                        callback_start_ns <= timestamp <= callback_end_ns
                        for timestamp in flow_packets
                    )
                else:
                    within_count = len(flow_packets)
                count_error = abs(within_count - expected_callback_count)
            if (
                expected_callback_count is not None
                and callback_start_ns is not None
                and callback_end_ns is not None
            ):
                callback_duration = callback_end_ns - callback_start_ns
                flow_duration = flow_end - flow_start
                duration_error = abs(flow_duration - callback_duration)
                boundary_error = abs(flow_start - callback_start_ns) + abs(
                    flow_end - callback_end_ns
                )
            return (count_error, duration_error, boundary_error)

        selected = min(candidates, key=flow_score)
        selected_packets = [
            packet
            for packet in packets
            if (
                packet["source_host"],
                packet["source_port"],
                packet["destination_host"],
                packet["destination_port"],
            )
            == (
                selected["source_host"],
                selected["source_port"],
                selected["destination_host"],
                selected["destination_port"],
            )
        ]
        selected_packets.sort(key=lambda packet: packet["timestamp_ns"])
    else:
        selected_packets = []
    return flows, selected_packets


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round(fraction * (len(ordered) - 1))
    return ordered[max(0, min(index, len(ordered) - 1))]


def interval_metrics(
    timestamps_ns: list[int],
    *,
    warning_ms: float,
    fail_ms: float,
) -> dict[str, Any]:
    intervals = [
        (current - previous) / 1_000_000.0
        for previous, current in zip(timestamps_ns, timestamps_ns[1:])
        if current >= previous
    ]
    return {
        "count": len(timestamps_ns),
        "duration_s": (
            (timestamps_ns[-1] - timestamps_ns[0]) / 1_000_000_000.0
            if len(timestamps_ns) > 1
            else 0.0
        ),
        "interval_ms": {
            "p50": percentile(intervals, 0.50),
            "p95": percentile(intervals, 0.95),
            "p99": percentile(intervals, 0.99),
            "max": max(intervals, default=0.0),
            "over_warning": sum(value > warning_ms for value in intervals),
            "over_fail": sum(value >= fail_ms for value in intervals),
            "under_1ms": sum(value < 1.0 for value in intervals),
        },
    }


def read_callback_rows(path: Path) -> tuple[list[int], list[float]]:
    import csv

    timestamps: list[int] = []
    gaps: list[float] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                timestamps.append(int(row["wall_time_ns"]))
                gaps.append(float(row["arrival_interval_ms"]))
            except (KeyError, TypeError, ValueError):
                continue
    timestamps.sort()
    return timestamps, gaps


def analyze_capture(
    callbacks_ns: list[int],
    packet_timestamps_ns: list[int],
    *,
    expected_rate_hz: float,
    warning_ms: float,
    fail_ms: float,
) -> dict[str, Any]:
    callbacks = interval_metrics(
        callbacks_ns,
        warning_ms=warning_ms,
        fail_ms=fail_ms,
    )
    packets = interval_metrics(
        packet_timestamps_ns,
        warning_ms=warning_ms,
        fail_ms=fail_ms,
    )
    events: list[dict[str, Any]] = []
    local_delayed = 0
    source_silent = 0
    mixed = 0
    if callbacks_ns and packet_timestamps_ns:
        for previous, current in zip(callbacks_ns, callbacks_ns[1:]):
            gap_ms = (current - previous) / 1_000_000.0
            if gap_ms <= warning_ms:
                continue
            left = bisect.bisect_right(packet_timestamps_ns, previous)
            right = bisect.bisect_left(packet_timestamps_ns, current)
            packets_in_window = packet_timestamps_ns[left:right]
            packets_during_gap = len(packets_in_window)
            expected_packets = max(
                1.0,
                gap_ms
                * max(1.0, expected_rate_hz)
                / 1_000.0,
            )
            coverage = packets_during_gap / expected_packets
            first_packet_offset_ms = (
                (packets_in_window[0] - previous) / 1_000_000.0
                if packets_in_window
                else gap_ms
            )
            last_packet_offset_ms = (
                (packets_in_window[-1] - previous) / 1_000_000.0
                if packets_in_window
                else gap_ms
            )
            packet_arrival_span_ms = (
                (packets_in_window[-1] - packets_in_window[0]) / 1_000_000.0
                if packets_in_window
                else 0.0
            )
            packet_silences: list[float] = []
            if left > 0 and packets_in_window:
                packet_silences.append(
                    (packets_in_window[0] - packet_timestamps_ns[left - 1])
                    / 1_000_000.0
                )
            packet_silences.extend(
                (following - preceding) / 1_000_000.0
                for preceding, following in zip(
                    packets_in_window, packets_in_window[1:]
                )
            )
            if packets_in_window:
                packet_silences.append(
                    (current - packets_in_window[-1]) / 1_000_000.0
                )
            max_packet_silence_ms = max(packet_silences, default=gap_ms)
            packets_are_distributed = (
                coverage >= 0.4
                and max_packet_silence_ms < max(warning_ms, gap_ms * 0.5)
                and first_packet_offset_ms < gap_ms * 0.5
            )
            packets_are_end_burst = (
                packets_during_gap > 0
                and (
                    max_packet_silence_ms >= gap_ms * 0.5
                    or first_packet_offset_ms >= gap_ms * 0.5
                )
            )
            if packets_are_distributed:
                classification = "LOCAL_CALLBACK_BATCHING"
                local_delayed += 1
            elif packets_during_gap == 0 or packets_are_end_burst:
                classification = "SOURCE_OR_WIRE_SILENCE"
                source_silent += 1
            else:
                classification = "MIXED"
                mixed += 1
            events.append(
                {
                    "callback_time_ns": current,
                    "callback_gap_ms": gap_ms,
                    "packets_during_gap": packets_during_gap,
                    "expected_packets": expected_packets,
                    "packet_coverage": coverage,
                    "first_packet_offset_ms": first_packet_offset_ms,
                    "last_packet_offset_ms": last_packet_offset_ms,
                    "packet_arrival_span_ms": packet_arrival_span_ms,
                    "max_packet_silence_ms": max_packet_silence_ms,
                    "classification": classification,
                }
            )

    callback_fail = callbacks["interval_ms"]["over_fail"]
    packet_fail = packets["interval_ms"]["over_fail"]
    if events:
        if local_delayed >= max(source_silent, mixed) and local_delayed >= math.ceil(
            len(events) * 0.6
        ):
            verdict = "LOCAL_CALLBACK_BATCHING"
        elif source_silent >= max(local_delayed, mixed) and source_silent >= math.ceil(
            len(events) * 0.6
        ):
            verdict = "SOURCE_OR_WIRE_DELAY"
        else:
            verdict = "MIXED_OR_INCONCLUSIVE"
    elif callback_fail == 0 and packet_fail == 0:
        verdict = "NO_LONG_GAPS"
    else:
        verdict = "INCONCLUSIVE"

    return {
        "verdict": verdict,
        "callbacks": callbacks,
        "packets": packets,
        "callback_long_gap_events": events,
        "classification_counts": {
            "local_callback_batching": local_delayed,
            "source_or_wire_silence": source_silent,
            "mixed": mixed,
        },
    }


def find_tcpdump(requested: Path | None) -> str:
    path = str(requested.resolve()) if requested is not None else shutil.which(
        "tcpdump"
    )
    if path is None:
        raise SystemExit(
            "tcpdump is required. Install it with: sudo apt install tcpdump"
        )
    if not Path(path).is_file():
        raise SystemExit(f"tcpdump executable does not exist: {path}")
    return path


def privilege_prefix(tool: str, no_sudo: bool) -> list[str]:
    if os.name != "posix" or os.geteuid() == 0 or no_sudo:
        return []
    sudo = shutil.which("sudo")
    if sudo is None:
        return []
    check = subprocess.run([sudo, "-v"], check=False)
    if check.returncode != 0:
        raise SystemExit("sudo authentication failed")
    return [sudo, "-n"]


def teardown_process(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def write_report(
    output_dir: Path,
    metadata: dict[str, Any],
    analysis: dict[str, Any] | None,
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    document = {"metadata": metadata, "analysis": analysis}
    json_path = output_dir / "wire_summary.json"
    text_path = output_dir / "wire_summary.txt"
    json_path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    lines = [
        f"WIRE VERDICT: {analysis['verdict'] if analysis else 'NO_ANALYSIS'}",
        f"endpoint: {metadata['endpoint']}",
        f"interface: {metadata['interface']}",
    ]
    if analysis:
        callbacks = analysis["callbacks"]["interval_ms"]
        packets = analysis["packets"]["interval_ms"]

        def gap(value: float | None) -> str:
            return "n/a" if value is None else f"{value:.3f}"

        lines.extend(
            [
                "callback gap p50/p95/p99/max: "
                f"{gap(callbacks['p50'])} / {gap(callbacks['p95'])} / "
                f"{gap(callbacks['p99'])} / {gap(callbacks['max'])} ms",
                "packet gap p50/p95/p99/max: "
                f"{gap(packets['p50'])} / {gap(packets['p95'])} / "
                f"{gap(packets['p99'])} / {gap(packets['max'])} ms",
                "callback gaps >25/>50ms: "
                f"{callbacks['over_warning']}/{callbacks['over_fail']}",
                "packet gaps >25/>50ms: "
                f"{packets['over_warning']}/{packets['over_fail']}",
                "long-gap classifications: "
                f"{analysis['classification_counts']}",
            ]
        )
        if analysis["callback_long_gap_events"]:
            lines.append("largest callback gap events:")
            for event in sorted(
                analysis["callback_long_gap_events"],
                key=lambda item: item["callback_gap_ms"],
                reverse=True,
            )[:10]:
                lines.append(
                    "  "
                    f"gap={event['callback_gap_ms']:.2f}ms "
                    f"packets_in_gap={event['packets_during_gap']} "
                    f"coverage={event['packet_coverage']:.2f} "
                    f"class={event['classification']}"
                )
    ping = metadata.get("ping")
    if ping:
        lines.append(
            "ping: "
            f"loss={ping.get('packet_loss_percent')}% "
            f"avg={ping.get('rtt_avg_ms')}ms "
            f"max={ping.get('rtt_max_ms')}ms"
        )
    lines.extend(
        [
            f"pcap: {metadata.get('pcap')}",
            f"stability CSV: {metadata.get('stability_csv')}",
        ]
    )
    text_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, text_path


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture VRPN callbacks and UDP packets simultaneously, then "
            "classify whether long callback gaps occur before or after the "
            "host UDP receive point."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--endpoint")
    parser.add_argument("--interface")
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--gap-warning-ms", type=float, default=25.0)
    parser.add_argument("--gap-fail-ms", type=float, default=50.0)
    parser.add_argument("--label", default="wire")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--stability-script", type=Path, default=DEFAULT_STABILITY)
    parser.add_argument("--probe", type=Path, default=DEFAULT_PROBE)
    parser.add_argument(
        "--tcpdump",
        type=Path,
        help="Alternative tcpdump executable path.",
    )
    parser.add_argument(
        "--no-sudo",
        action="store_true",
        help="Do not use sudo; tcpdump must already have capture capability.",
    )
    parser.add_argument(
        "--skip-ping",
        action="store_true",
        help="Do not run a concurrent continuous ping.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    config = load_config(args.config)
    vrpn = config.get("vrpn", {})
    control = config.get("control", {})
    if not isinstance(vrpn, dict) or not isinstance(control, dict):
        raise SystemExit("config vrpn/control sections must be objects")
    endpoint = args.endpoint or str(
        vrpn.get("endpoint", "MCAvatar@192.168.31.221:3883")
    )
    _, target_host, _ = endpoint_parts(endpoint)
    expected_rate = float(vrpn.get("expected_rate_hz", 120.0))
    interface = args.interface or detect_interface(target_host)
    output_dir = (
        args.output_dir
        or REPO_ROOT
        / "logs"
        / "vrpn-wire"
        / f"run_{utc_label()}_{args.label}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    stability_dir = output_dir / "stability"
    pcap_path = output_dir / "capture.pcap"
    packet_text_path = output_dir / "packets.txt"
    capture_log_path = output_dir / "tcpdump.log"
    capture_tool = find_tcpdump(args.tcpdump)
    prefix = privilege_prefix(capture_tool, args.no_sudo)

    filter_expression = f"udp and host {target_host}"
    capture_command = [
        *prefix,
        capture_tool,
        "-i",
        interface,
        "-nn",
        "-s",
        "0",
        "-U",
        "--time-stamp-precision=nano",
        "-w",
        str(pcap_path),
        filter_expression,
    ]
    stability_command = [
        sys.executable,
        str(args.stability_script.resolve()),
        "--config",
        str(args.config.resolve()),
        "--endpoint",
        endpoint,
        "--duration",
        str(args.duration),
        "--probe",
        str(args.probe.resolve()),
        "--output-dir",
        str(stability_dir),
        "--label",
        args.label,
        "--skip-ping",
    ]

    metadata: dict[str, Any] = {
        "schema": 1,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "endpoint": endpoint,
        "target_host": target_host,
        "interface": interface,
        "duration_requested_s": args.duration,
        "filter": filter_expression,
        "capture_command": capture_command,
        "stability_command": stability_command,
        "pcap": str(pcap_path),
    }

    capture_log = capture_log_path.open("w", encoding="utf-8")
    capture: subprocess.Popen[str] | None = None
    ping_process: subprocess.Popen[bytes] | None = None
    probe_return_code: int | None = None
    try:
        print("starting packet capture:", " ".join(capture_command))
        capture = subprocess.Popen(
            capture_command,
            cwd=REPO_ROOT,
            stdout=capture_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if capture.poll() is not None:
                raise SystemExit(
                    "tcpdump exited during startup; see "
                    f"{capture_log_path}"
                )
            if pcap_path.exists() and pcap_path.stat().st_size > 24:
                break
            time.sleep(0.1)
        else:
            raise SystemExit(
                f"packet capture did not become ready; see {capture_log_path}"
            )

        if not args.skip_ping:
            ping_path = output_dir / "ping-concurrent.log"
            ping_count = max(1, round(args.duration / 0.2))
            ping_command = [
                "ping",
                "-i",
                "0.2",
                "-c",
                str(ping_count),
                target_host,
            ]
            metadata["ping_command"] = ping_command
            metadata["ping_log"] = str(ping_path)
            try:
                with ping_path.open("wb") as ping_log:
                    ping_process = subprocess.Popen(
                        ping_command,
                        stdout=ping_log,
                        stderr=subprocess.STDOUT,
                    )
            except OSError as exception:
                metadata["ping_error"] = str(exception)

        print("starting synchronized VRPN probe")
        probe_return_code = subprocess.run(
            stability_command,
            cwd=REPO_ROOT,
            check=False,
        ).returncode
    except KeyboardInterrupt:
        probe_return_code = 130
        print("\ninterrupted; preserving partial capture", file=sys.stderr)
    finally:
        teardown_process(ping_process)
        teardown_process(capture)
        capture_log.close()

    metadata["probe_return_code"] = probe_return_code
    metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
    if ping_process is not None and ping_process.returncode is not None:
        metadata["ping_exit_code"] = ping_process.returncode
        metadata["ping"] = parse_ping_output(output_dir / "ping-concurrent.log")

    stability_csv = stability_dir / "vrpn_samples.csv"
    metadata["stability_csv"] = str(stability_csv)
    if not pcap_path.exists() or pcap_path.stat().st_size <= 24:
        raise SystemExit(f"packet capture is empty: {pcap_path}")
    if not stability_csv.exists():
        raise SystemExit(f"VRPN callback CSV is missing: {stability_csv}")

    decode_command = [
        *prefix,
        capture_tool,
        "-nn",
        "-tt",
        "--time-stamp-precision=nano",
        "-r",
        str(pcap_path),
    ]
    decoded = subprocess.run(
        decode_command,
        capture_output=True,
        text=True,
        check=False,
    )
    packet_text_path.write_text(
        (decoded.stdout or "") + (decoded.stderr or ""),
        encoding="utf-8",
    )
    if decoded.returncode != 0:
        raise SystemExit(f"cannot decode capture; see {packet_text_path}")

    callback_timestamps, _ = read_callback_rows(stability_csv)
    flows, packets = parse_packet_dump(
        decoded.stdout,
        target_host,
        expected_callback_count=len(callback_timestamps),
        callback_start_ns=(
            callback_timestamps[0] if callback_timestamps else None
        ),
        callback_end_ns=(
            callback_timestamps[-1] if callback_timestamps else None
        ),
    )
    packet_timestamps = [packet["timestamp_ns"] for packet in packets]
    analysis = analyze_capture(
        callback_timestamps,
        packet_timestamps,
        expected_rate_hz=expected_rate,
        warning_ms=args.gap_warning_ms,
        fail_ms=args.gap_fail_ms,
    )
    selected_flow = None
    if packets:
        selected_key = (
            packets[0]["source_host"],
            packets[0]["source_port"],
            packets[0]["destination_host"],
            packets[0]["destination_port"],
        )
        selected_flow = next(
            (
                flow
                for flow in flows
                if (
                    flow["source_host"],
                    flow["source_port"],
                    flow["destination_host"],
                    flow["destination_port"],
                )
                == selected_key
            ),
            None,
        )
    analysis["selected_flow"] = selected_flow
    analysis["flows"] = flows[:20]
    json_path, text_path = write_report(output_dir, metadata, analysis)
    print()
    print(text_path.read_text(encoding="utf-8"), end="")
    print(f"json: {json_path}")
    print(f"pcap: {pcap_path}")
    return 1 if analysis["verdict"] in {
        "SOURCE_OR_WIRE_DELAY",
        "LOCAL_CALLBACK_BATCHING",
        "MIXED_OR_INCONCLUSIVE",
        "INCONCLUSIVE",
    } else 0


if __name__ == "__main__":
    raise SystemExit(main())
