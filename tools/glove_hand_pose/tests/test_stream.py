"""Raw I/O tests; no serial hardware or Qt is used."""

import csv
from dataclasses import FrozenInstanceError
import errno
import importlib.util
import os
from pathlib import Path
import random
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, PropertyMock, patch

from vt_glove_pose.records import Sample
from vt_glove_pose.stream import (
    CSV_FIELDS, MAX_READ_BYTES, ImuParser, RawRecorder, ReplaySource,
    SerialSource, StreamError, iter_csv, list_serial_ports,
    _claim_tty_exclusive, _find_serial_port_holders, _release_tty_exclusive,
)


VALUES = (-32768, 32767, -257, 256, -1, 0)


def frame(first_id=1):
    return b"".join(struct.pack("<BB6h", node, node + 100, *VALUES)
                    for node in range(first_id, first_id + 6))


def sample(node=1, mono=2.0, wall=100.0, status=None):
    return Sample(node, VALUES, mono, wall, status)


class SampleTests(unittest.TestCase):
    def test_frozen_and_values_are_immutable(self):
        values = list(VALUES)
        record = Sample(1, values, 1, 2, 0)
        values[0] = 0
        self.assertEqual(record.values, VALUES)
        with self.assertRaises(FrozenInstanceError):
            record.node_id = 2

    def test_invalid_node_values_status_and_times(self):
        arguments = dict(node_id=1, values=VALUES, host_monotonic_s=1, host_wall_s=2)
        invalid = (
            ("node_id", 0), ("node_id", 13), ("node_id", True), ("node_id", 1.5),
            ("values", (0,) * 5), ("values", (0,) * 7), ("values", None),
            ("values", (32768, 0, 0, 0, 0, 0)),
            ("values", (True, 0, 0, 0, 0, 0)),
            ("values", (1.0, 0, 0, 0, 0, 0)),
            ("status", -1), ("status", 256), ("status", True), ("status", 1.5),
            ("host_monotonic_s", float("nan")), ("host_wall_s", float("inf")),
            ("host_monotonic_s", "1.0"), ("host_wall_s", True),
            ("host_monotonic_s", 10**500),
        )
        for name, value in invalid:
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                Sample(**(arguments | {name: value}))


class ParserTests(unittest.TestCase):
    def test_little_endian_both_groups_status_and_host_times(self):
        parser = ImuParser()
        records = parser.feed(frame() + frame(7), 4.25, 100.5)
        self.assertEqual([r.node_id for r in records], list(range(1, 13)))
        self.assertTrue(all(r.values == VALUES for r in records))
        self.assertEqual([r.status for r in records], list(range(101, 113)))
        self.assertTrue(all(r.host_monotonic_s == 4.25 for r in records))
        self.assertTrue(all(r.host_wall_s == 100.5 for r in records))
        self.assertEqual(parser.frames_ok, 2)
        self.assertEqual(parser.bytes_discarded, 0)
        self.assertEqual(parser.buffer, b"")

    def test_every_single_split_and_one_byte_fragments(self):
        data = frame() + frame(7)
        for split in range(len(data) + 1):
            with self.subTest(split=split):
                parser = ImuParser()
                records = parser.feed(data[:split], 1, 2) + parser.feed(data[split:], 1, 2)
                self.assertEqual(len(records), 12)
                self.assertEqual(parser.frames_ok, 2)
                self.assertEqual(parser.bytes_discarded, 0)
        parser = ImuParser()
        records = [record for byte in data for record in parser.feed(bytes([byte]), 1, 2)]
        self.assertEqual(len(records), 12)
        self.assertEqual(parser.buffer, b"")

    def test_random_fragmentation_noise_corruption_and_resynchronization(self):
        damaged = bytearray(frame())
        damaged[28] = 99
        noise = b"bad prefix" + bytes(damaged) + b"\xff\xfe\xff"
        data = noise + frame(7) + b"between" + frame()
        parser = ImuParser()
        records = []
        rng = random.Random(1234)
        cursor = 0
        while cursor < len(data):
            size = rng.randint(1, 43)
            records.extend(parser.feed(data[cursor:cursor + size], 1, 2))
            cursor += size
            self.assertLessEqual(len(parser.buffer), 83)
        self.assertEqual([r.node_id for r in records], list(range(7, 13)) + list(range(1, 7)))
        self.assertEqual(parser.bytes_discarded, len(noise) + len(b"between"))
        self.assertEqual(parser.frames_ok, 2)

    def test_large_noise_is_bounded_and_later_recovered(self):
        parser = ImuParser()
        noise = b"\xff" * 100_000
        self.assertEqual(parser.feed(noise, 1, 2), [])
        self.assertEqual(len(parser.buffer), 83)
        self.assertEqual(parser.bytes_discarded, len(noise) - 83)
        self.assertEqual(len(parser.feed(frame(), 3, 4)), 6)
        self.assertEqual(parser.bytes_discarded, len(noise))
        self.assertEqual(parser.buffer, b"")

    def test_large_valid_input_and_buffer_protocol(self):
        parser = ImuParser()
        self.assertEqual(len(parser.feed(memoryview(frame() * 100), 1, 2)), 600)
        self.assertEqual(parser.frames_ok, 100)
        self.assertEqual(parser.bytes_discarded, 0)
        self.assertEqual(parser.buffer, b"")

    def test_partial_frame_uses_completion_parse_time(self):
        parser = ImuParser()
        self.assertEqual(parser.feed(frame()[:40], 1, 2), [])
        records = parser.feed(frame()[40:], 3, 4)
        self.assertTrue(all(r.host_monotonic_s == 3 and r.host_wall_s == 4 for r in records))

    def test_default_clocks_and_reset(self):
        parser = ImuParser()
        with patch("vt_glove_pose.stream.time.monotonic", return_value=9), patch(
            "vt_glove_pose.stream.time.time", return_value=10
        ):
            records = parser.feed(b"noise" + frame() + b"partial")
        self.assertEqual(records[0], sample(mono=9, wall=10, status=101))
        parser.reset()
        self.assertEqual(parser.frames_ok, 0)
        self.assertEqual(parser.bytes_discarded, 0)
        self.assertEqual(parser.buffer, b"")

    def test_invalid_timestamp_rejected_for_empty_input(self):
        with self.assertRaises(ValueError):
            ImuParser().feed(b"", float("nan"), 1)


class CsvTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "raw.csv"

    def write_text(self, text):
        self.path.write_text(text, encoding="utf-8")

    def write_rows(self, rows, fields=CSV_FIELDS):
        with self.path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(fields)
            writer.writerows(rows)

    def test_roundtrip_exact_schema_and_optional_status(self):
        records = [sample(status=0), sample(mono=2, status=255), sample(12, 3, 99)]
        with RawRecorder(self.path) as recorder:
            recorder.write(iter(records))
            self.assertEqual(list(iter_csv(self.path)), records)  # flushed each batch
        self.assertEqual(self.path.read_text().splitlines()[0], ",".join(CSV_FIELDS))
        self.assertEqual(list(iter_csv(self.path)), records)

    def test_exclusive_creation_does_not_overwrite(self):
        self.write_text("do not alter")
        with self.assertRaises(FileExistsError):
            RawRecorder(self.path).open()
        self.assertEqual(self.path.read_text(), "do not alter")

    def test_recorder_lifecycle_and_reject_non_sample(self):
        recorder = RawRecorder(self.path)
        self.assertFalse(self.path.exists())
        with self.assertRaises(RuntimeError):
            recorder.write([])
        self.assertIs(recorder.open(), recorder)
        self.assertIs(recorder.open(), recorder)
        with self.assertRaises(TypeError):
            recorder.write([None])
        recorder.close()
        recorder.close()
        with self.assertRaises(RuntimeError):
            recorder.write([])
        with self.assertRaises(RuntimeError):
            recorder.open()

    def test_optional_phase_without_status(self):
        self.write_rows([(2, 100, 1, *VALUES, "neutral")], CSV_FIELDS[:-1] + ("phase",))
        self.assertEqual(list(iter_csv(self.path)), [sample()])

    def test_duplicate_host_times_and_wall_clock_adjustment_are_valid(self):
        self.write_rows([(2, 100, 1, *VALUES, ""), (2, 99, 1, *VALUES, "")])
        self.assertEqual(len(list(iter_csv(self.path))), 2)

    def test_time_order_is_per_node(self):
        self.write_rows([(3, 100, 1, *VALUES, ""), (2, 99, 2, *VALUES, "")])
        self.assertEqual(len(list(iter_csv(self.path))), 2)
        self.write_rows([(3, 100, 1, *VALUES, ""), (2, 101, 1, *VALUES, "")])
        with self.assertRaisesRegex(ValueError, "line 3: nonmonotonic.*node 1"):
            list(iter_csv(self.path))

    def test_invalid_fields_and_timestamps_have_line_context(self):
        base = [2, 100, 1, *VALUES, ""]
        for index, value in ((0, "nan"), (1, "inf"), (2, "0"), (2, "1.5"),
                             (3, "32768"), (4, "oops"), (9, "256"), (0, "")):
            with self.subTest(index=index, value=value):
                row = base.copy()
                row[index] = value
                self.write_rows([row])
                with self.assertRaisesRegex(ValueError, "line 2"):
                    list(iter_csv(self.path))

    def test_short_and_long_rows_rejected(self):
        for row in ((2, 100, 1), (2, 100, 1, *VALUES, "", "extra")):
            with self.subTest(row=row):
                self.write_rows([row])
                with self.assertRaisesRegex(ValueError, "field count"):
                    list(iter_csv(self.path))

    def test_missing_duplicate_and_unexpected_headers_rejected(self):
        for fields in (CSV_FIELDS[1:], CSV_FIELDS + ("node_id",), CSV_FIELDS + ("typo",)):
            with self.subTest(fields=fields):
                self.write_rows([], fields)
                with self.assertRaises(ValueError):
                    list(iter_csv(self.path))
        self.write_text("")
        with self.assertRaisesRegex(ValueError, "no header"):
            list(iter_csv(self.path))

    def test_malformed_csv_quote_rejected(self):
        self.write_text(",".join(CSV_FIELDS) + '\n"unterminated')
        with self.assertRaisesRegex(ValueError, "invalid CSV"):
            list(iter_csv(self.path))
        self.write_text('"unterminated header')
        with self.assertRaisesRegex(ValueError, "invalid CSV header"):
            list(iter_csv(self.path))

    def test_empty_valid_csv(self):
        self.write_rows([])
        self.assertEqual(list(iter_csv(self.path)), [])


class ReplaySourceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "raw.csv"

    def record(self, records):
        with RawRecorder(self.path) as recorder:
            recorder.write(records)

    def test_pacing_speed_preserved_timestamps_and_eof(self):
        records = [sample(mono=10), sample(2, mono=10), sample(mono=12), sample(mono=14)]
        self.record(records)
        clock = Mock(return_value=100.0)
        with patch("vt_glove_pose.stream.time.monotonic", clock), ReplaySource(self.path, speed=2) as replay:
            self.assertFalse(replay.eof)
            self.assertEqual(replay.read(), records[:2])
            self.assertEqual(replay.read(), [])
            clock.return_value = 100.99
            self.assertEqual(replay.read(), [])
            clock.return_value = 101.0
            self.assertEqual(replay.read(), records[2:3])
            clock.return_value = 102.0
            self.assertEqual(replay.read(), records[3:])
            self.assertTrue(replay.eof)
            self.assertEqual(replay.read(), [])

    def test_backlog_is_bounded_without_dropping_samples(self):
        records = [sample(mono=10)] * 600
        self.record(records)
        with ReplaySource(self.path) as replay:
            self.assertEqual(len(replay.read()), 256)
            self.assertFalse(replay.eof)
            self.assertEqual(len(replay.read()), 256)
            self.assertEqual(len(replay.read()), 88)
            self.assertTrue(replay.eof)

    def test_empty_csv_lifecycle_and_reopen(self):
        self.record([])
        replay = ReplaySource(self.path)
        with self.assertRaises(RuntimeError):
            replay.read()
        self.assertIs(replay.open(), replay)
        self.assertIs(replay.open(), replay)
        self.assertTrue(replay.eof)
        self.assertEqual(replay.read(), [])
        replay.close()
        replay.close()
        with self.assertRaises(RuntimeError):
            replay.read()
        replay.open()
        replay.close()

    def test_invalid_options(self):
        for speed in (0, -1, float("inf"), float("nan"), True, "1", 10**500):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                ReplaySource(self.path, speed)
        for count in (0, -1, 1.5, True):
            with self.subTest(count=count), self.assertRaises(ValueError):
                ReplaySource(self.path, max_samples=count)

    def test_invalid_csv_and_missing_file_fail_on_open(self):
        with self.assertRaises(FileNotFoundError):
            ReplaySource(self.path).open()
        self.path.write_text("invalid\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            ReplaySource(self.path).open()


class SerialTests(unittest.TestCase):
    def setUp(self):
        # Unit-test devices are mocks, never real terminals or /proc targets.
        for name, result in (("_find_serial_port_holders", []), ("_claim_tty_exclusive", False)):
            patcher = patch(f"vt_glove_pose.stream.{name}", return_value=result)
            patcher.start()
            self.addCleanup(patcher.stop)

    def module(self):
        class SerialException(OSError):
            pass
        return types.SimpleNamespace(SerialException=SerialException, Serial=Mock())

    def test_explicit_port_short_timeout_exclusive_and_bounded_reads(self):
        serial = self.module()
        device = serial.Serial.return_value
        device.in_waiting = 1_000_000
        device.read.return_value = frame()
        source = SerialSource("/dev/explicit-imu")
        serial.Serial.assert_not_called()
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), source:
            kwargs = {"timeout": 0.020, "write_timeout": 0.2}
            if os.name == "posix":
                kwargs["exclusive"] = True
            serial.Serial.assert_called_once_with("/dev/explicit-imu", 115200, **kwargs)
            self.assertEqual(len(source.read()), 6)
            device.read.assert_called_once_with(MAX_READ_BYTES)
            device.write.assert_not_called()
            device.reset_input_buffer.assert_not_called()
        device.close.assert_called_once()

    def test_idle_reads_request_one_byte_and_source_lifecycle(self):
        serial = self.module()
        device = serial.Serial.return_value
        device.in_waiting = 0
        device.read.return_value = b""
        source = SerialSource("chosen")
        with self.assertRaises(RuntimeError):
            source.read()
        with patch("vt_glove_pose.stream._serial_module", return_value=serial):
            self.assertIs(source.open(), source)
            source.open()
            self.assertEqual(source.read(), [])
        serial.Serial.assert_called_once()
        device.read.assert_called_once_with(1)
        source.close()
        source.close()
        device.close.assert_called_once()
        with self.assertRaises(RuntimeError):
            source.read()

    def test_busy_and_disconnection_are_controlled_errors(self):
        serial = self.module()
        serial.Serial.side_effect = serial.SerialException("Resource temporarily unavailable")
        with patch("vt_glove_pose.stream._serial_module", return_value=serial):
            with self.assertRaisesRegex(StreamError, "close any monitor"):
                SerialSource("busy").open()
        serial.Serial.side_effect = None
        device = serial.Serial.return_value
        device.in_waiting = 1
        device.read.side_effect = serial.SerialException("disconnected")
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), SerialSource("chosen") as source:
            with self.assertRaisesRegex(StreamError, "Serial read failed"):
                source.read()

    def test_invalid_port_and_baud(self):
        for port in (None, "", "   "):
            with self.subTest(port=port), self.assertRaises(ValueError):
                SerialSource(port)
        for baud in (0, -1, True, 115200.0):
            with self.subTest(baud=baud), self.assertRaises(ValueError):
                SerialSource("chosen", baud)

    def test_discovery_lists_choices_without_opening(self):
        serial = self.module()
        ports = types.SimpleNamespace(comports=lambda: [
            types.SimpleNamespace(device="/dev/ttyACM1", description="IMU"),
            types.SimpleNamespace(device="/dev/ttyACM0", description="pressure"),
        ])
        serial_tools = types.ModuleType("serial.tools")
        serial_tools.list_ports = ports
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), patch.dict(
            "sys.modules", {"serial.tools": serial_tools}
        ):
            self.assertEqual(list_serial_ports(), [("/dev/ttyACM0", "pressure"), ("/dev/ttyACM1", "IMU")])
        serial.Serial.assert_not_called()

    def test_host_queue_and_read_metrics_are_bounded_detached_and_reset_on_open(self):
        serial = self.module()
        device = serial.Serial.return_value
        device.read.side_effect = [frame(), frame(7)]
        source = SerialSource("chosen")
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), patch.object(
            type(device), "in_waiting", new_callable=PropertyMock, create=True
        ) as waiting, patch("vt_glove_pose.stream.time.perf_counter") as clock:
            waiting.side_effect = [5000, 100, 84, 0]
            clock.side_effect = [1, 1.002, 1.003, 1.004, 2, 2.001, 2.002, 2.004]
            source.open()
            self.assertEqual(source.performance["raw_read_calls"], 0)
            self.assertEqual(len(source.read()), 6)
            first = source.performance
            self.assertEqual(first["serial_backlog_before_bytes"], 5000)
            self.assertEqual(first["serial_backlog_after_bytes"], 100)
            self.assertEqual(first["serial_backlog_peak_bytes"], 5000)
            self.assertAlmostEqual(first["raw_read_elapsed_ms"], 2)
            self.assertAlmostEqual(first["parse_elapsed_ms"], 1)
            self.assertEqual(first["raw_read_bytes"], 84)
            self.assertEqual(first["raw_read_bytes_total"], 84)
            self.assertEqual(first["parser_frames_ok"], 1)
            self.assertEqual(len(source.read()), 6)
            second = source.performance
            self.assertEqual(second["serial_backlog_before_bytes"], 84)
            self.assertEqual(second["serial_backlog_after_bytes"], 0)
            self.assertEqual(second["serial_backlog_peak_bytes"], 5000)
            self.assertEqual(second["raw_read_calls"], 2)
            self.assertEqual(second["raw_read_bytes_total"], 168)
            self.assertEqual(second["parser_frames_ok"], 2)
            self.assertEqual(set(first), set(second))
            self.assertTrue(all(not isinstance(value, (list, dict, tuple)) for value in second.values()))
            second["raw_read_bytes_total"] = -1
            self.assertEqual(source.performance["raw_read_bytes_total"], 168)
            self.assertEqual(waiting.call_count, 4)  # Inspecting metrics never queries the port.
            source.close()
            source.open()
            self.assertEqual(source.performance["raw_read_calls"], 0)
            self.assertEqual(source.performance["serial_backlog_peak_bytes"], 0)
            source.close()

    def test_optional_post_read_queue_failure_does_not_lose_completed_samples(self):
        serial = self.module()
        device = serial.Serial.return_value
        device.read.return_value = frame()
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), patch.object(
            type(device), "in_waiting", new_callable=PropertyMock, create=True
        ) as waiting, SerialSource("chosen") as source:
            waiting.side_effect = [84, OSError("optional queue observation failed")]
            self.assertEqual(len(source.read()), 6)
            self.assertIsNone(source.performance["serial_backlog_after_bytes"])
            self.assertEqual(source.performance["serial_backlog_peak_bytes"], 84)
            self.assertEqual(source.performance["raw_read_calls"], 1)


class SerialSafetyTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform.startswith("linux"), "Linux /proc preflight")
    def test_visible_proc_holders_resolve_aliases_and_include_own_process(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            device = root / "tty-test"
            device.touch()
            alias = root / "by-id-test"
            alias.symlink_to(device)
            for pid, fd in ((12345, 4), (os.getpid(), 9)):
                descriptors = root / "proc" / str(pid) / "fd"
                descriptors.mkdir(parents=True, exist_ok=True)
                (descriptors / str(fd)).symlink_to(device)
            other = root / "proc" / "45678" / "fd"
            other.mkdir(parents=True)
            (other / "3").symlink_to(root / "unrelated")
            (other / "4").symlink_to("socket:[1234]")
            (root / "proc" / "self").mkdir()
            self.assertEqual(
                _find_serial_port_holders(str(alias), proc_root=root / "proc"),
                sorted([(12345, 4), (os.getpid(), 9)]),
            )

    def test_existing_holder_is_rejected_before_any_serial_open(self):
        serial = types.SimpleNamespace(Serial=Mock(), SerialException=OSError)
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), patch(
            "vt_glove_pose.stream._find_serial_port_holders", return_value=[(12345, 7)]
        ):
            with self.assertRaisesRegex(StreamError, "PID 12345.*Close the owning application"):
                SerialSource("selected-port").open()
        serial.Serial.assert_not_called()

    @unittest.skipUnless(os.name == "posix", "POSIX tty ioctls")
    def test_claim_release_call_tty_ioctls_without_writing_device(self):
        import termios
        if not hasattr(termios, "TIOCEXCL"):
            self.skipTest("TIOCEXCL not available")
        device = Mock()
        device.fileno.return_value = 42
        with patch("fcntl.ioctl") as ioctl:
            self.assertTrue(_claim_tty_exclusive(device))
            ioctl.assert_called_once_with(42, termios.TIOCEXCL)
            if hasattr(termios, "TIOCNXCL"):
                _release_tty_exclusive(device)
                self.assertEqual(ioctl.call_args.args, (42, termios.TIOCNXCL))
        device.write.assert_not_called()

    def test_claim_failure_closes_open_descriptor_and_reports_controlled_error(self):
        serial = types.SimpleNamespace(Serial=Mock(), SerialException=OSError)
        device = serial.Serial.return_value
        source = SerialSource("selected-port")
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), patch(
            "vt_glove_pose.stream._find_serial_port_holders", return_value=[]
        ), patch("vt_glove_pose.stream._claim_tty_exclusive", side_effect=OSError("exclusive ioctl failed")):
            with self.assertRaisesRegex(StreamError, "exclusive ioctl failed"):
                source.open()
        device.close.assert_called_once()
        self.assertIsNone(source._serial)

    def test_close_releases_claim_then_closes_and_is_idempotent(self):
        serial = types.SimpleNamespace(Serial=Mock(), SerialException=OSError)
        source = SerialSource("selected-port")
        with patch("vt_glove_pose.stream._serial_module", return_value=serial), patch(
            "vt_glove_pose.stream._find_serial_port_holders", return_value=[]
        ), patch("vt_glove_pose.stream._claim_tty_exclusive", return_value=True), patch(
            "vt_glove_pose.stream._release_tty_exclusive"
        ) as release:
            source.open()
            source.close()
            source.close()
        release.assert_called_once_with(serial.Serial.return_value)
        serial.Serial.return_value.close.assert_called_once()

    @unittest.skipUnless(sys.platform.startswith("linux") and os.getuid() != 0, "Unprivileged Linux PTY test")
    def test_real_pty_blocks_preexisting_and_subsequent_noncooperating_readers(self):
        # This opens only a newly allocated pseudoterminal, never glove hardware.
        if importlib.util.find_spec("serial") is None:
            self.skipTest("pyserial not installed")
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        name = os.ttyname(slave)
        try:
            with self.assertRaisesRegex(StreamError, "already open"):
                SerialSource(name).open()  # Detect even this process's existing slave fd.
        finally:
            os.close(slave)
        with SerialSource(name):
            with self.assertRaises(OSError) as caught:
                # A noncooperating ordinary opener does not need to use flock.
                duplicate = os.open(name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                os.close(duplicate)
            self.assertEqual(caught.exception.errno, errno.EBUSY)
        # Release is explicit: the still-open PTY master must not strand the lock.
        reopened = os.open(name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(reopened)


if __name__ == "__main__":
    unittest.main()
