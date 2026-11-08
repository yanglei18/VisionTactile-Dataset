"""No hardware required: wire format, boundaries, corruption and buffering."""

import random
import struct

import pytest

from vt_tactile_capture.protocol import Block, FRAME_SIZE, FramingEvidence, Parser, frame_key


def packet(row=0, adc=None):
    values = tuple(range(20)) if adc is None else tuple(adc)
    return b"\xa5\x5a" + struct.pack(">H20H", row, *values)


@pytest.mark.parametrize("split", range(FRAME_SIZE + 1))
def test_every_packet_split(split):
    wire = packet(19, (0, 65535, 255, 256, *range(16)))
    parser = Parser()
    blocks = parser.feed(wire[:split]) + parser.feed(wire[split:])
    assert blocks == (Block(19, (0, 65535, 255, 256, *range(16))),)
    assert parser.buffered_bytes == 0
    assert parser.discarded_bytes_total == parser.malformed_packets_total == 0


def test_multiple_packets_and_byte_at_a_time_preserve_order():
    wire = b"".join(packet(row) for row in range(20))
    expected = Parser().feed(wire)
    parser = Parser(max_buffer_bytes=44)
    actual = tuple(block for value in wire for block in parser.feed(bytes([value])))
    assert actual == expected
    assert len(actual) == 20


def test_header_in_payload_is_not_treated_as_a_new_packet():
    assert Parser().feed(packet(adc=[0xA55A] * 20))[0].adc == (0xA55A,) * 20


def test_repeated_header_prefix_and_noise_are_counted():
    parser = Parser()
    assert parser.feed(b"junk\xa5") == ()
    assert parser.buffered_bytes == 1
    assert parser.feed(packet(2))[0].row == 2
    assert parser.discarded_bytes_total == 5
    assert parser.malformed_packets_total == 0


@pytest.mark.parametrize("bad", [packet(20), packet(80), packet(65535)])
def test_invalid_addresses_recover_and_count(bad):
    parser = Parser()
    assert parser.feed(bad + packet(7)) == (Block(7, tuple(range(20))),)
    assert parser.malformed_packets_total == 1
    assert parser.discarded_bytes_total == 44


def test_invalid_candidate_does_not_swallow_overlapping_real_header():
    parser = Parser()
    prefix = b"\xa5\x5a\xff\xff"
    assert parser.feed(prefix + packet(4))[0].row == 4
    assert parser.malformed_packets_total == 1
    assert parser.discarded_bytes_total == len(prefix)


def test_large_noisy_input_and_random_chunking_are_bounded():
    random_source = random.Random(9041)
    wire = b"\x11" * 100_000 + b"".join(packet(row) for row in range(20))
    parser = Parser(max_buffer_bytes=49)
    actual = []
    offset = 0
    while offset < len(wire):
        count = random_source.randint(1, 3017)
        actual.extend(parser.feed(wire[offset:offset + count]))
        assert parser.buffered_bytes < FRAME_SIZE
        offset += count
    assert len(actual) == 20
    assert parser.discarded_bytes_total == 100_000
    assert parser.malformed_packets_total == 0


def test_reset_discards_partial_frame_without_joining_different_connections():
    parser = Parser()
    parser.feed(packet()[:19])
    parser.reset()
    assert parser.buffered_bytes == 0
    assert parser.discarded_bytes_total == 19
    assert parser.feed(packet(3))[0].row == 3


def test_too_small_buffer_rejected():
    with pytest.raises(ValueError):
        Parser(max_buffer_bytes=43)


@pytest.mark.parametrize("chunk_size", [1, 2, 43, 44, 45, 4096])
def test_shared_framing_evidence_accepts_only_consecutive_complete_rows(chunk_size):
    evidence = FramingEvidence()
    data = b"noise" + b"".join(packet(row) for row in range(8))
    for offset in range(0, len(data), chunk_size):
        evidence.feed(data[offset:offset + chunk_size])
    assert evidence.confirmed
    assert frame_key(packet(0)) == 0
    assert frame_key(packet(19)) == 19
    assert frame_key(packet(20)) is None
    assert frame_key(packet(0)[:-1]) is None


def test_one_row_or_interrupted_run_does_not_confirm_protocol():
    assert not FramingEvidence().feed(packet(0) * 20)
    assert not FramingEvidence().feed(b"".join(packet(row) + b"noise" for row in range(20)))


@pytest.mark.parametrize("low_byte", [0, 3, 7])
def test_old_26_byte_stream_never_confirms_row44(low_byte):
    data = b"".join(b"\xa5\x5a" + struct.pack(">HBB10H", row, 0, low_byte, *range(10))
                    for row in range(20))
    assert not FramingEvidence().feed(data)


@pytest.mark.parametrize("chunk_size", [1, 43, 44, 45, 4096])
def test_confirmed_stream_batches_retain_read_clocks_through_assembler(chunk_size):
    from itertools import groupby
    from vt_tactile_capture.protocol import StreamParser
    from vt_tactile_capture.assembler import Assembler
    from vt_tactile_capture.serial_source import Batch

    wire = b"".join(packet(i % 20, [i] * 20) for i in range(400)) + packet(0)[:4]
    parser, assembler = StreamParser(), Assembler()
    packet_index = sequence = 0
    last_time = -1
    for read, offset in enumerate(range(0, len(wire), chunk_size)):
        stamp = 1000 + read
        outputs = parser.feed(wire[offset:offset + chunk_size], stamp)
        for source_time, group in groupby(outputs, key=lambda value: value[1]):
            blocks = tuple(block for block, _ in group)
            assert source_time > last_time
            last_time = source_time
            assembler.apply(Batch(session_id="test", stream_epoch=1, sequence=sequence,
                first_packet_index=packet_index, host_realtime_ns=source_time,
                host_monotonic_ns=source_time, connected=True, time_valid=True,
                reason="ok", blocks=blocks))
            state = assembler.snapshot(source_time, 1000000)
            assert state.time_valid and state.reason == "ok"
            packet_index += len(blocks)
            sequence += 1
    assert packet_index == 400
    assert sum(state.valid_mask) == 400
    assert list(state.adc) == [380 + row for row in range(20) for col in range(20)]
    assert parser.buffered_bytes == 4


def test_old_prefix_and_midstream_protocol_change_never_admit_false_rows():
    from vt_tactile_capture.protocol import StreamParser
    old = b"".join(b"\xa5\x5a" + struct.pack(">HBB10H", row, 0, 0, *range(10))
                    for row in range(10))
    new = b"".join(packet(row, [20000 + row] * 20) for row in range(8))
    parser = StreamParser()
    result = parser.feed(old + new + packet(8)[:4], 1)
    assert [b.row for b, _ in result] == list(range(8))
    assert [b.adc[0] for b, _ in result] == list(range(20000, 20008))
    assert parser.discarded_bytes_total == len(old)
    parser = StreamParser()
    first = parser.feed(new, 1)
    second = parser.feed(old, 2)
    assert [b.row for b, _ in first + second] == list(range(8))
    assert not parser.confirmed
    resumed = parser.feed(new + packet(8)[:4], 3)
    assert [b.adc[0] for b, _ in resumed] == list(range(20000, 20008))


def test_confirmed_stream_discarded_tail_and_read_size_are_bounded():
    from vt_tactile_capture.protocol import StreamParser
    parser = StreamParser()
    wire = b"".join(packet(row) for row in range(8))
    assert not parser.feed(wire, 1)  # One read's final frame is not yet confirmed.
    parser.reset()
    assert parser.discarded_bytes_total == len(wire)
    assert parser.buffered_bytes == 0 and not parser.confirmed
    with pytest.raises(ValueError, match="byte cap"):
        parser.feed(b"x" * 4097, 2)
