import csv

import pytest

from vt_fsglove12.live_replay import PacedCSVSource


def raw(path):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["host_parse_monotonic_s", "host_parse_wall_s", "node_id",
                         "gx_lsb", "gy_lsb", "gz_lsb", "ax_lsb", "ay_lsb", "az_lsb", "status"])
        for t in (15., 15.1):
            for node in range(1, 13):
                writer.writerow([t, t + 100, node, 1, 2, 3, 4, 5, 8197, 127])


def test_paced_source_retains_raw_values_and_eof_never_loops(tmp_path):
    path = tmp_path / "raw.csv"
    raw(path)
    original = path.read_bytes()
    now = [100.]
    source = PacedCSVSource(path, clock=lambda: now[0], wall_clock=lambda: 500.)
    source.open()
    first = source.read_samples()
    assert len(first) == 12
    assert {s.host_monotonic_s for s in first} == {100.}
    assert first[4].node_id == 5 and first[4].values == (1, 2, 3, 4, 5, 8197)
    assert first[4].status == 127 and first[4].host_wall_s == 500.
    assert source.read_samples() == []
    now[0] = 100.1
    second = source.read_samples()
    assert len(second) == 12 and source.eof
    assert all(s.host_monotonic_s == pytest.approx(100.1) for s in second)
    now[0] = 900.
    assert source.read_samples() == []
    source.close()
    assert path.read_bytes() == original


@pytest.mark.parametrize("speed", [0, -1, float("nan"), float("inf"), True])
def test_bad_speed_is_not_a_live_source(speed):
    with pytest.raises(ValueError):
        PacedCSVSource("unused.csv", speed=speed)
