"""Optional resistance estimate using the supplied 20by20.py circuit settings.

The wire carries ADC counts. These constants describe the reference program,
not a measured circuit calibration or a conversion to force/pressure. A missing
resistance estimate never invalidates the original ADC sample.
"""
from numbers import Integral


ADC_MAX = 65535
ADC_VREF_V = 3.3
OFFSET_V = 0.9
FIXED_RESISTANCE_OHM = 20_000.0


def adc_to_resistance(adc: int) -> float | None:
    """Return ohms, or None when the reference circuit formula is undefined."""
    if isinstance(adc, bool) or not isinstance(adc, Integral) or not 0 <= adc <= ADC_MAX:
        raise ValueError("ADC must be an unsigned 16-bit integer")
    denominator = int(adc) * ADC_VREF_V / ADC_MAX - OFFSET_V
    if denominator <= 0:
        return None
    return OFFSET_V * FIXED_RESISTANCE_OHM / denominator
