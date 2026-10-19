"""No hardware required: wire format, boundaries, corruption and buffering."""

import random
import struct

import pytest

from vt_tactile_capture.protocol import Block, FRAME_SIZE, Parser


def packet(row=0, group=0, status=0x73, adc=None):
    values = tuple(range(10)) if adc is None else tuple(adc)
    return b"\xa5\x5a" + struct.pack(">HBB10H", row, status, group, *values)


@pytest.mark.parametrize("split", range(FRAME_SIZE + 1))
def test_every_packet_split(split):
    wire = packet(79, 7, 0xE3, (0, 65535, 255, 256, 1, 2, 3, 4, 5, 6))
    parser = Parser()
    blocks = parser.feed(wire[:split]) + parser.feed(wire[split:])
    assert blocks == (Block(79, 7, 0xE3, (0, 65535, 255, 256, 1, 2, 3, 4, 5, 6)),)
    assert parser.buffered_bytes == 0
    assert parser.discarded_bytes_total == parser.malformed_packets_total == 0


def test_multiple_packets_and_byte_at_a_time_preserve_order():
    wire = b"".join(packet(row, row % 8) for row in range(80))
    expected = Parser().feed(wire)
    parser = Parser(max_buffer_bytes=26)
    actual = tuple(block for value in wire for block in parser.feed(bytes([value])))
    assert actual == expected
    assert len(actual) == 80


def test_header_in_payload_is_not_treated_as_a_new_packet():
    assert Parser().feed(packet(adc=[0xA55A] * 10))[0].adc == (0xA55A,) * 10


def test_repeated_header_prefix_and_noise_are_counted():
    parser = Parser()
    assert parser.feed(b"junk\xa5") == ()
    assert parser.buffered_bytes == 1
    assert parser.feed(packet(2, 3))[0].row == 2
    assert parser.discarded_bytes_total == 5
    assert parser.malformed_packets_total == 0


@pytest.mark.parametrize("bad", [packet(80, 0), packet(65535, 0), packet(0, 8), packet(0, 255)])
def test_invalid_addresses_recover_and_count(bad):
    parser = Parser()
    assert parser.feed(bad + packet(7, 6)) == (Block(7, 6, 0x73, tuple(range(10))),)
    assert parser.malformed_packets_total == 1
    assert parser.discarded_bytes_total == 26


def test_invalid_candidate_does_not_swallow_overlapping_real_header():
    parser = Parser()
    prefix = b"\xa5\x5a\xff\xff"
    assert parser.feed(prefix + packet(4, 2))[0].row == 4
    assert parser.malformed_packets_total == 1
    assert parser.discarded_bytes_total == len(prefix)


def test_large_noisy_input_and_random_chunking_are_bounded():
    random_source = random.Random(9041)
    wire = b"\x11" * 100_000 + b"".join(packet(row, row % 8) for row in range(80))
    parser = Parser(max_buffer_bytes=31)
    actual = []
    offset = 0
    while offset < len(wire):
        count = random_source.randint(1, 3017)
        actual.extend(parser.feed(wire[offset:offset + count]))
        assert parser.buffered_bytes < FRAME_SIZE
        offset += count
    assert len(actual) == 80
    assert parser.discarded_bytes_total == 100_000
    assert parser.malformed_packets_total == 0


def test_reset_discards_partial_frame_without_joining_different_connections():
    parser = Parser()
    parser.feed(packet()[:19])
    parser.reset()
    assert parser.buffered_bytes == 0
    assert parser.discarded_bytes_total == 19
    assert parser.feed(packet(3, 1))[0].row == 3


def test_too_small_buffer_rejected():
    with pytest.raises(ValueError):
        Parser(max_buffer_bytes=25)
