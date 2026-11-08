"""Independent byte-by-byte reference from the supplied row44 specification."""
import random
from vt_tactile_capture.protocol import Parser


def test_reference_wire_and_matrix_order():
    rng = random.Random(20261029)
    matrix = [[rng.randrange(65536) for _ in range(20)] for _ in range(20)]
    wire = bytearray()
    for row, values in enumerate(matrix):
        wire.extend((0xA5, 0x5A, 0, row))
        for value in values:
            wire.extend((value // 256, value % 256))
    packets = Parser().feed(bytes(wire))
    assert [p.row for p in packets] == list(range(20))
    assert [list(p.adc) for p in packets] == matrix
