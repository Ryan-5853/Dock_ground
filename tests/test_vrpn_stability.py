#!/usr/bin/env python3

import csv
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "test_vrpn_stability.py"
SPEC = importlib.util.spec_from_file_location("vrpn_stability", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
VRPN_STABILITY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VRPN_STABILITY)


def write_fixture(path: Path, *, bad_gap: bool, bad_jump: bool) -> None:
    fields = [
        "schema",
        "record_index",
        "received_mono_ns",
        "wall_time_ns",
        "sensor",
        "source_time_ns",
        "source_time_present",
        "source_clock_delta_ms",
        "arrival_interval_ms",
        "source_time_interval_ms",
        "source_time_regression",
        "raw_x",
        "raw_y",
        "raw_z",
        "qx",
        "qy",
        "qz",
        "qw",
        "qnorm",
    ]
    start = 1_000_000_000
    received = start
    source = 1_700_000_000_000_000_000
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(400):
            gap_ms = 8
            if index == 200 and bad_gap:
                gap_ms = 150
            if index:
                received += gap_ms * 1_000_000
                source += gap_ms * 1_000_000
            x = 10 if index <= 200 or not bad_jump else 200
            writer.writerow(
                {
                    "schema": 1,
                    "record_index": index,
                    "received_mono_ns": received,
                    "wall_time_ns": received,
                    "sensor": 2,
                    "source_time_ns": source,
                    "source_time_present": 1,
                    "source_clock_delta_ms": 0,
                    "arrival_interval_ms": -1 if index == 0 else gap_ms,
                    "source_time_interval_ms": -1 if index == 0 else gap_ms,
                    "source_time_regression": 0,
                    "raw_x": x,
                    "raw_y": 20,
                    "raw_z": 1500,
                    "qx": 0,
                    "qy": 0,
                    "qz": 0,
                    "qw": 1,
                    "qnorm": 1,
                }
            )


def analyze(path: Path):
    return VRPN_STABILITY.analyze_csv(
        path,
        expected_rate_hz=120.0,
        position_scale=0.001,
        gap_warning_ms=25.0,
        gap_fail_ms=100.0,
        max_pose_age_ms=50.0,
        min_rate_ratio=0.8,
        static_test=True,
        max_position_step_m=0.05,
        max_rotation_step_deg=20.0,
    )


class VrpnStabilityAnalysisTest(unittest.TestCase):
    def test_clean_stream_passes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.csv"
            write_fixture(path, bad_gap=False, bad_jump=False)
            result = analyze(path)
        self.assertEqual(result["verdict"], "PASS")
        self.assertEqual(result["arrival_gap_ms"]["over_warning"], 0)
        self.assertEqual(result["pose_quality"]["max_position_step_m"], 0.0)

    def test_gap_and_pose_jump_fail(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.csv"
            write_fixture(path, bad_gap=True, bad_jump=True)
            result = analyze(path)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertEqual(result["arrival_gap_ms"]["over_fail"], 1)
        self.assertTrue(
            any("maximum callback gap" in item for item in result["failures"])
        )
        self.assertTrue(
            any("static position step" in item for item in result["failures"])
        )

    def test_quaternion_angle_ignores_sign(self) -> None:
        angle = VRPN_STABILITY.quaternion_angle_deg(
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, -1.0),
        )
        self.assertIsNotNone(angle)
        self.assertTrue(math.isclose(float(angle), 0.0, abs_tol=1e-9))


if __name__ == "__main__":
    unittest.main()
