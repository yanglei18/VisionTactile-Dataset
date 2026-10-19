"""Default board geometry is a shared, editable configuration contract."""


def test_default_board_matches_cc200_pattern_dimensions():
    from vt_capture_core.calibration_policy import board_geometry_defaults

    board = board_geometry_defaults()
    assert board == dict(squares_x=12, squares_y=9, square_length_mm=15.,
                         marker_length_mm=11.25, dictionary="DICT_5X5_1000")
    assert board["squares_x"] * board["square_length_mm"] == 180.
    assert board["squares_y"] * board["square_length_mm"] == 135.
    assert (board["squares_x"] - 1) * (board["squares_y"] - 1) == 88


def test_editing_one_board_draft_does_not_change_new_session_defaults():
    from vt_capture_core.calibration_policy import board_geometry_defaults

    draft = board_geometry_defaults()
    draft.update(squares_x=9, square_length_mm=40.)
    assert board_geometry_defaults()["squares_x"] == 12
    assert board_geometry_defaults()["square_length_mm"] == 15.
