"""Optional live-core/offline equivalence checks, without ROS or hardware."""

from dataclasses import fields
from types import SimpleNamespace as NS
import struct
import unittest

import numpy as np

from vt_multisensor_alignment.model import MessageRef
from vt_multisensor_alignment.tactile_adapter import TactileContinuity, TactileGrid


try:
    from vt_tactile_capture.assembler import Assembler
    from vt_tactile_capture.protocol import Block, Parser
    from vt_tactile_capture.serial_source import Batch
except ImportError:
    Assembler = None


@unittest.skipIf(Assembler is None, "add ros2_ws/src/vt_tactile_capture to PYTHONPATH for cross-path checks")
class TactileCrossPathTests(unittest.TestCase):
    def compare(self, batches, *, queries=None, max_age_ns=100):
        live = Assembler()
        offline = TactileGrid()
        continuity = TactileContinuity()
        for index, batch in enumerate(batches):
            live.apply(batch)
            message = NS(**{field.name: getattr(batch, field.name) for field in fields(Batch)})
            # Match generated uint16[10] ROS arrays at the decoder boundary.
            message.blocks = [NS(row=block.row, column_group=block.column_group,
                                 status_raw=block.status_raw, adc=np.asarray(block.adc, dtype=np.uint16))
                              for block in batch.blocks]
            event = continuity.decode(message, MessageRef("/test/tactile", index, 1000 + index,
                                                          batch.host_realtime_ns),
                                      header_stamp_ns=batch.host_realtime_ns)
            offline.apply(event)
            delay_ns = queries[index] if queries else 0
            snapshot = live.snapshot(batch.host_monotonic_ns + delay_ns, max_age_ns)
            grid = offline.snapshot(batch.host_realtime_ns + delay_ns, max_age_ns)
            self.assertEqual((snapshot.rows, snapshot.cols, snapshot.schema_version), (20, 20, 2))
            np.testing.assert_array_equal(np.asarray(snapshot.adc).reshape(20, 20), grid["adc"])
            np.testing.assert_array_equal(np.asarray(snapshot.valid_mask).reshape(20, 20), grid["valid_mask"])
            np.testing.assert_array_equal(np.asarray(snapshot.block_realtime_ns).reshape(20, 8),
                                          grid["block_realtime_ns"])
        return live, offline

    def batch(self, time_ns, sequence, *, blocks=None, **updates):
        values = dict(session_id="cross-test", connected=True, time_valid=True,
                      stream_epoch=1, sequence=sequence, first_packet_index=sequence,
                      host_realtime_ns=time_ns, host_monotonic_ns=time_ns + 1000,
                      blocks=tuple(blocks if blocks is not None else [Block(2, 3, 201, tuple(range(10)))]))
        values.update(updates)
        return Batch(**values)

    def test_multiple_host_reads_partial_coverage_age_and_valid_zero(self):
        live, offline = self.compare([
            self.batch(100, 0),
            self.batch(180, 1, blocks=[Block(79, 7, 255, (65535,) * 10)]),
            self.batch(210, 2, blocks=[Block(0, 0, 0, (0,) * 10)]),
        ], queries=[5, 5, 0], max_age_ns=100)
        grid = offline.snapshot(210, 100)
        self.assertEqual(grid["valid_mask"].sum(), 3)
        self.assertFalse(grid["valid_mask"][2, 3])
        self.assertTrue(grid["valid_mask"][0, 0])
        self.assertEqual(grid["adc"][0, 0], 0)

    def test_full_legacy_scan_and_outside_roi_only_batch(self):
        scan = [Block(row, group, 255,
                      tuple(row * 80 + group + 8 * k for k in range(10)))
                for row in range(80) for group in range(8)]
        _, offline = self.compare([
            self.batch(100, 0, blocks=scan),
            self.batch(140, 1, first_packet_index=640,
                       blocks=[Block(79, 7, 255, (65535,) * 10)]),
        ], queries=[0, 0], max_age_ns=100)
        grid = offline.snapshot(140, 100)
        self.assertEqual(grid["valid_mask"].sum(), 400)
        np.testing.assert_array_equal(grid["adc"],
                                      np.arange(6400).reshape(80, 80)[:20, :20])
        np.testing.assert_array_equal(grid["block_realtime_ns"], np.full((20, 8), 100))

    def test_epoch_and_transport_loss_boundaries_clear_old_groups(self):
        for updates in ({"stream_epoch": 2}, {"sequence": 4},
                        {"first_packet_index": 9}, {"producer_dropped_total": 1},
                        {"discarded_bytes_total": 5}, {"malformed_packets_total": 1}):
            with self.subTest(updates=updates):
                values = dict(blocks=[Block(3, 4, 0, (42,) * 10)])
                values.update(updates)
                sequence = values.pop("sequence", 1)
                self.compare([self.batch(100, 0), self.batch(200, sequence, **values)])

    def test_status_events_invalidate_instead_of_creating_valid_zero(self):
        self.compare([
            self.batch(100, 0),
            self.batch(200, 1, blocks=[], connected=False, time_valid=False, reason="disconnect"),
            self.batch(300, 2, first_packet_index=1),
        ])

    def test_arbitrarily_split_protocol_packets_keep_one_read_completion_clock(self):
        wire = b"\xa5\x5a" + struct.pack(">HBB10H", 19, 173, 7, *range(10))
        parser = Parser()
        self.assertEqual(parser.feed(wire[:19]), ())
        decoded = parser.feed(wire[19:] + wire)
        self.assertEqual(len(decoded), 2)
        self.assertEqual(decoded[0].status_raw, 173)
        _, offline = self.compare([self.batch(100, 0, blocks=decoded)])
        self.assertEqual(offline.block_realtime_ns[19, 7], 100)
        np.testing.assert_array_equal(offline.adc[19, 7::8], np.arange(2))

    def test_new_session_and_epoch_can_restart_monotonic_origin(self):
        for updates in ({"session_id": "new-session"}, {"stream_epoch": 2}):
            with self.subTest(updates=updates):
                self.compare([self.batch(100, 0),
                              self.batch(200, 1, host_monotonic_ns=1, **updates)])

    def test_invalid_domains_duplicate_clocks_and_clock_steps_clear_both_paths(self):
        for updates in ({"clock_domain": "unsupported"}, {"timing_source": "publish_time"},
                        {"host_monotonic_ns": 1100}, {"host_realtime_ns": 100},
                        {"host_monotonic_ns": 20_000_000}):
            with self.subTest(updates=updates):
                time_ns = updates.get("host_realtime_ns", 200)
                values = {name: value for name, value in updates.items() if name != "host_realtime_ns"}
                self.compare([self.batch(100, 0), self.batch(time_ns, 1, **values)])

    def test_counter_regressions_clear_both_paths(self):
        for sequence, updates in ((0, {}), (1, {"first_packet_index": 0}),
                                  (1, {"stream_epoch": 0})):
            with self.subTest(sequence=sequence, updates=updates):
                self.compare([self.batch(100, 0), self.batch(200, sequence, **updates)])


if __name__ == "__main__":
    unittest.main()
