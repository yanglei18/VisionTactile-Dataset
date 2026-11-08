"""Shared tactile display palettes with no numerical or GUI dependencies."""

RAW_HEAT_STOPS = ((0., (237, 244, 252)), (.32, (136, 198, 251)),
                 (.58, (14, 124, 237)), (.78, (249, 191, 101)), (1., (238, 105, 77)))
RESPONSE_HEAT_STOPS = ((0., (204, 236, 205)), (.35, (246, 232, 137)),
                      (.7, (236, 136, 72)), (1., (150, 30, 40)))


def heat_stops(value_kind="raw_adc"):
    """Keep matrix, hand and 3D views consistent for each explicit value kind."""
    if value_kind == "raw_adc":
        return RAW_HEAT_STOPS
    if value_kind == "tactile_response":
        return RESPONSE_HEAT_STOPS
    raise ValueError("Unknown tactile display value kind")
