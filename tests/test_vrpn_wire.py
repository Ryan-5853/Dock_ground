#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "test_vrpn_wire.py"
SPEC = importlib.util.spec_from_file_location("vrpn_wire", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
VRPN_WIRE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VRPN_WIRE)


def regular_timestamps(count: int, period_ns: int = 8_333_333) -> list[int]:
    start = 1_000_000_000
    return [start + index * period_ns for index in range(count)]


def with_gap(
    timestamps: list[int],
    index: int,
    gap_ns: int,
) -> list[int]:
    result = list(timestamps)
    for offset in range(index, len(result)):
        result[offset] += gap_ns
    return result


class VrpnWireAnalysisTest(unittest.TestCase):
    def test_endpoint_parts(self) -> None:
        self.assertEqual(
            VRPN_WIRE.endpoint_parts("MCAvatar@192.168.31.221:3883"),
            ("MCAvatar", "192.168.31.221", 3883),
        )

    def test_packet_parser_selects_tracker_flow(self) -> None:
        text = "\n".join(
            [
                "1793089226.869174975 IP 192.168.31.221.43001 > "
                "192.168.31.220.50000: UDP, length 72",
                "1793089226.877508308 IP 192.168.31.221.43001 > "
                "192.168.31.220.50000: UDP, length 72",
                "1793089226.878000000 IP 192.168.31.220.50000 > "
                "192.168.31.221.43001: UDP, length 20",
            ]
        )
        flows, packets = VRPN_WIRE.parse_packet_dump(
            text, "192.168.31.221"
        )
        self.assertEqual(len(packets), 2)
        self.assertEqual(flows[0]["source_host"], "192.168.31.221")
        self.assertEqual(packets[1]["length"], 72)

    def test_packet_parser_prefers_callback_matching_flow(self) -> None:
        text = "\n".join(
            [
                "1700000000.000000000 IP 192.168.31.221.43001 > "
                "192.168.31.220.50000: UDP, length 72",
                "1700000000.050000000 IP 192.168.31.221.43001 > "
                "192.168.31.220.50000: UDP, length 72",
                "1700000000.100000000 IP 192.168.31.221.43001 > "
                "192.168.31.220.50000: UDP, length 72",
                "1700000000.150000000 IP 192.168.31.221.43001 > "
                "192.168.31.220.50000: UDP, length 72",
                "1700000000.001000000 IP 192.168.31.221.43002 > "
                "192.168.31.220.50001: UDP, length 72",
                "1700000000.051000000 IP 192.168.31.221.43002 > "
                "192.168.31.220.50001: UDP, length 72",
                "1700000000.101000000 IP 192.168.31.221.43002 > "
                "192.168.31.220.50001: UDP, length 72",
            ]
        )
        _, packets = VRPN_WIRE.parse_packet_dump(
            text,
            "192.168.31.221",
            expected_callback_count=3,
            callback_start_ns=1_700_000_000_000_000_000,
            callback_end_ns=1_700_000_000_150_000_000,
        )
        self.assertEqual(len(packets), 3)
        self.assertEqual(packets[0]["source_port"], 43002)

    def test_continuous_packets_with_delayed_callbacks_are_local(self) -> None:
        packets = regular_timestamps(120)
        callbacks = with_gap(packets, 60, 100_000_000)
        analysis = VRPN_WIRE.analyze_capture(
            callbacks,
            packets,
            expected_rate_hz=120.0,
            warning_ms=25.0,
            fail_ms=50.0,
        )
        self.assertEqual(analysis["verdict"], "LOCAL_CALLBACK_BATCHING")
        self.assertGreater(
            analysis["classification_counts"]["local_callback_batching"], 0
        )

    def test_packet_and_callback_gap_are_upstream(self) -> None:
        packets = with_gap(regular_timestamps(120), 60, 100_000_000)
        analysis = VRPN_WIRE.analyze_capture(
            packets,
            packets,
            expected_rate_hz=120.0,
            warning_ms=25.0,
            fail_ms=50.0,
        )
        self.assertEqual(analysis["verdict"], "SOURCE_OR_WIRE_DELAY")
        self.assertGreater(
            analysis["classification_counts"]["source_or_wire_silence"], 0
        )

    def test_packets_bursting_at_end_of_gap_are_upstream(self) -> None:
        callbacks = with_gap(regular_timestamps(120), 60, 100_000_000)
        packets = with_gap(regular_timestamps(120), 60, 100_000_000)
        # Put eight packets immediately before the delayed callback. Counting
        # them is not enough: their compressed arrival span must not be
        # mistaken for continuous packet delivery.
        burst_start = callbacks[60] - 8 * 100_000
        packets[53:61] = [
            burst_start + index * 100_000 for index in range(8)
        ]
        analysis = VRPN_WIRE.analyze_capture(
            callbacks,
            packets,
            expected_rate_hz=120.0,
            warning_ms=25.0,
            fail_ms=50.0,
        )
        self.assertEqual(analysis["verdict"], "SOURCE_OR_WIRE_DELAY")


if __name__ == "__main__":
    unittest.main()
