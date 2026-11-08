"""Reference circuit boundaries, independent of transport or display scaling."""
import pytest

from vt_tactile_capture.conversion import adc_to_resistance


@pytest.mark.parametrize("adc", [0, 100, 17873])
def test_legal_adc_can_have_no_resistance_estimate(adc):
    assert adc_to_resistance(adc) is None


def test_reference_formula_and_threshold():
    assert adc_to_resistance(17874) > 0
    assert adc_to_resistance(65535) == pytest.approx(7500)
    assert adc_to_resistance(18837) == pytest.approx(370882.8522920205)
    assert adc_to_resistance(18195) > adc_to_resistance(18837)
    # The reference application's color scale is not a physical range limit.
    assert adc_to_resistance(18837) > 24000


@pytest.mark.parametrize("adc", [-1, 65536, True, False, 1.5, None, "18000"])
def test_invalid_adc_is_not_silently_clipped(adc):
    with pytest.raises(ValueError):
        adc_to_resistance(adc)
