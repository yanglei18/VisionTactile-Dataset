import numpy as np
import pytest

from vt_tactile_capture.presentation import delta_color_stops, region_summaries


def test_delta_lut_neutral_band_and_signed_ends():
    positions, colors = delta_color_stops((-1000, 16000))
    assert positions.dtype == np.float64
    assert colors.dtype == np.uint8
    assert colors.shape == (len(positions), 4)
    assert positions[0] == 0 and positions[-1] == 1
    assert np.all(np.diff(positions) > 0)
    np.testing.assert_array_equal(colors[0], (49, 121, 192, 255))
    np.testing.assert_array_equal(colors[-1], (240, 92, 52, 255))
    for delta in (-200, 0, 200):
        color = [np.interp((delta + 1000) / 17000, positions, colors[:, c]) for c in range(4)]
        np.testing.assert_allclose(color, (155, 174, 185, 255))


@pytest.mark.parametrize("levels", [(0, 100), (-100, 0), (-100, 100), (20, 80)])
def test_range_inside_deadband_is_all_neutral(levels):
    positions, colors = delta_color_stops(levels)
    np.testing.assert_array_equal(positions, (0, 1))
    np.testing.assert_array_equal(colors, [(155, 174, 185, 255)] * 2)


@pytest.mark.parametrize("levels", [(1000, 2000), (-2000, -1000), (200, 400), (-400, -200), (0, 1000)])
def test_ranges_not_containing_zero_keep_strict_endpoints(levels):
    positions, colors = delta_color_stops(levels)
    assert np.all(np.diff(positions) > 0)
    assert positions[0] == 0 and positions[-1] == 1
    assert np.all(colors[:, 3] == 255)
    if levels[0] > 200:
        assert np.all(colors[:, 0] > colors[:, 2])  # Positive, not arbitrarily blue.
    if levels[1] < -200:
        assert np.all(colors[:, 2] > colors[:, 0])


def test_zero_deadband_and_asymmetric_limits_do_not_duplicate_stops():
    for levels, deadband in [((-10, 10), 0), ((-1e-10, 1e10), 0), ((-1e10, 1e-10), 0),
                             ((-200, 200), 200), ((-10, 10), 1000)]:
        positions, colors = delta_color_stops(levels, deadband)
        assert positions[0] == 0 and positions[-1] == 1
        assert np.all(np.diff(positions) > 0)
        assert len(colors) == len(positions)


@pytest.mark.parametrize("levels", [(1, 1), (2, 1), (np.nan, 1), (0, np.inf), (1,), [1, 2, 3],
                                   (-1e308, 1e308), ("bad", 2)])
def test_bad_levels_rejected(levels):
    with pytest.raises(ValueError):
        delta_color_stops(levels)


@pytest.mark.parametrize("value", [-1, np.nan, np.inf, True, [], "bad"])
def test_bad_deadband_or_threshold_rejected(value):
    with pytest.raises(ValueError):
        delta_color_stops((-1, 1), value)
    with pytest.raises(ValueError):
        region_summaries(np.zeros((20, 20)), np.ones((20, 20), dtype=bool), value)


def test_region_negative_peak_missing_cells_and_strict_threshold():
    values = np.zeros((20, 20))
    valid = np.zeros((20, 20), dtype=bool)
    values[8, 16:20] = (200, -200, 201, -500)
    valid[8, 16:20] = True
    values[9, 16] = 65535  # Invalid large value cannot win.
    values[9, 17:19] = (np.nan, np.inf)
    valid[9, 17:19] = True
    summary = region_summaries(values, valid)
    thumb = summary[0]
    assert thumb["peak_delta"] == -500
    assert thumb["positive_peak"] == 201
    assert thumb["negative_peak"] == -500
    assert thumb["active_cells"] == 2
    assert thumb["valid_cells"] == 4
    assert thumb["candidate_cells"] == 48
    assert summary[1]["peak_delta"] is None
    assert summary[1]["valid_cells"] == 0
    assert summary[1]["active_cells"] == 0


def test_valid_zero_is_distinguished_from_no_measurements():
    values = np.zeros((20, 20))
    valid = np.zeros((20, 20), dtype=bool)
    valid[18, 1] = True
    summary = region_summaries(values, valid)
    assert summary[4]["peak_delta"] == 0
    assert summary[4]["valid_cells"] == 1
    assert summary[4]["active_cells"] == 0
    assert summary[0]["peak_delta"] is None


def test_candidate_regions_cover_400_cells_exactly_once_without_mutation():
    values = np.ones((20, 20))
    valid = np.ones((20, 20), dtype=bool)
    original_values, original_valid = values.copy(), valid.copy()
    result = region_summaries(values, valid, threshold=0)
    assert [item["key"] for item in result] == [
        "thumb", "index", "middle", "ring", "little", "palm_center", "palm_root"]
    assert [item["candidate_cells"] for item in result] == [48] * 5 + [100, 60]
    assert sum(item["candidate_cells"] for item in result) == 400
    for row in range(20):
        for col in range(20):
            single = np.zeros((20, 20), dtype=bool)
            single[row, col] = True
            assert sum(item["valid_cells"] for item in region_summaries(values, single)) == 1
    valid[:20, :20] = False
    assert sum(item["valid_cells"] for item in region_summaries(values, valid)) == 0
    np.testing.assert_array_equal(values, original_values)
    valid[:20, :20] = True
    np.testing.assert_array_equal(valid, original_valid)


@pytest.mark.parametrize("values,mask", [
    (np.zeros((80, 80)), np.zeros((20, 20), dtype=bool)),
    (np.zeros((20, 20)), np.zeros((80, 80), dtype=bool)),
    (np.zeros((20, 20)), np.zeros((20, 20), dtype=np.uint8)),
    (["invalid"], np.zeros((20, 20), dtype=bool)),
])
def test_invalid_summary_arrays(values, mask):
    with pytest.raises(ValueError):
        region_summaries(values, mask)
