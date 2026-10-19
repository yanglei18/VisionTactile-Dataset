"""Default calibration policy shared by configuration builders and guided tasks.

Every session freezes these values in its own configuration. Existing explicit
configuration files retain their values; changing defaults never rewrites them.
"""

GUIDED_VALIDATION_PAIRS = 10
GUIDED_ROUNDS = 3


def board_geometry_defaults():
    """Editable CC200-15-11.25 board inputs, with lengths in millimetres.

    The 12 x 9 pattern measures 180 x 135 mm. The 200 x 150 mm backing
    includes a margin and must not be used to scale the detected corners.
    Imported configurations keep their explicit geometry and dictionary.
    """
    return dict(squares_x=12, squares_y=9, square_length_mm=15.,
                marker_length_mm=11.25, dictionary="DICT_5X5_1000")


def board_detection_defaults():
    return dict(min_corners=12, max_reprojection_rms_px=2.)


def pairing_defaults():
    return dict(max_interpolation_gap_ms=50., stability_window_ms=100.,
                max_static_translation_m=.003, max_static_rotation_deg=.5,
                min_time_separation_s=.5, min_pose_translation_m=.020,
                min_pose_rotation_deg=5.)


def acceptance_defaults():
    return dict(min_pairs=40, holdout_fraction=.20,
                target_translation_rms_m=.005, target_rotation_rms_deg=.5,
                maximum_translation_rms_m=.010, maximum_rotation_rms_deg=1.)
