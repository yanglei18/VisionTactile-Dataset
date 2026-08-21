from dataclasses import FrozenInstanceError, fields
from pathlib import Path

import pytest
import yaml

from vt_realsense_capture.config import load_config


CONFIG = Path(__file__).parents[1] / "config" / "cameras.yaml"
EXAMPLE_CONFIG = CONFIG.with_name("cameras.example.yaml")

LOCKED_FIELD_MUTATIONS = [
    pytest.param(("cameras", 0, "model_token"), "D404", id="d405-1-model"),
    pytest.param(
        ("cameras", 0, "color_module"), "rgb_camera", id="d405-1-color-module"
    ),
    pytest.param(("cameras", 1, "model_token"), "D404", id="d405-2-model"),
    pytest.param(
        ("cameras", 1, "color_module"), "rgb_camera", id="d405-2-color-module"
    ),
    pytest.param(("cameras", 2, "model_token"), "D435", id="d436-model"),
    pytest.param(
        ("cameras", 2, "color_module"), "depth_module", id="d436-color-module"
    ),
    pytest.param(("stream", "width"), 640, id="stream-width"),
    pytest.param(("stream", "height"), 480, id="stream-height"),
    pytest.param(("stream", "fps"), 60, id="stream-fps"),
    pytest.param(("stream", "color_format"), "BGR8", id="stream-color-format"),
    pytest.param(
        ("stream", "color_encoding"), "bgr8", id="stream-color-encoding"
    ),
    pytest.param(("stream", "depth_format"), "Y16", id="stream-depth-format"),
    pytest.param(
        ("stream", "depth_encoding"), "mono16", id="stream-depth-encoding"
    ),
    pytest.param(
        ("recording", "max_bag_duration_seconds"),
        301,
        id="recording-max-bag-duration-seconds",
    ),
    pytest.param(
        ("recording", "max_bag_size_bytes"),
        137438953471,
        id="recording-max-bag-size-bytes",
    ),
    pytest.param(
        ("recording", "max_cache_size_bytes"),
        1073741823,
        id="recording-max-cache-size-bytes",
    ),
]

NUMERIC_FIELD_PATHS = [
    pytest.param(("stream", "width"), id="stream-width"),
    pytest.param(("stream", "height"), id="stream-height"),
    pytest.param(("stream", "fps"), id="stream-fps"),
    pytest.param(
        ("recording", "max_bag_duration_seconds"),
        id="recording-max-bag-duration-seconds",
    ),
    pytest.param(
        ("recording", "max_bag_size_bytes"), id="recording-max-bag-size-bytes"
    ),
    pytest.param(
        ("recording", "max_cache_size_bytes"),
        id="recording-max-cache-size-bytes",
    ),
]

NON_FINITE_VALUES = [
    pytest.param(float("nan"), id="nan"),
    pytest.param(float("inf"), id="positive-inf"),
    pytest.param(float("-inf"), id="negative-inf"),
]


def _write_modified_config(tmp_path: Path, old: str, new: str) -> Path:
    path = tmp_path / "bad.yaml"
    path.write_text(CONFIG.read_text().replace(old, new, 1))
    return path


def _write_document(tmp_path: Path, document: object) -> Path:
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def _set_path(document: object, parts: tuple[object, ...], value: object) -> None:
    target = document
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = value


def test_reference_inventory_is_preserved_and_unique():
    cfg = load_config(CONFIG)

    assert [
        (
            camera.name,
            camera.model_token,
            camera.serial,
            camera.firmware,
            camera.asic_serial,
            camera.color_module,
        )
        for camera in cfg.cameras
    ] == [
        (
            "d405_1",
            "D405",
            "260322278433",
            "5.15.1.55",
            "255323071625",
            "depth_module",
        ),
        (
            "d405_2",
            "D405",
            "260322276463",
            "5.15.1.55",
            "255323071742",
            "depth_module",
        ),
        (
            "d436",
            "D436",
            "408322071716",
            "5.17.0.213",
            "343123151280",
            "rgb_camera",
        ),
    ]
    assert len({camera.name for camera in cfg.cameras}) == 3
    assert len({camera.serial for camera in cfg.cameras}) == 3


def test_equivalent_camera_inventory_is_configurable():
    reference = load_config(CONFIG)
    example = load_config(EXAMPLE_CONFIG)

    assert tuple(
        (camera.model_token, camera.color_module)
        for camera in example.cameras
    ) == (
        ("D405", "depth_module"),
        ("D405", "depth_module"),
        ("D436", "rgb_camera"),
    )
    assert {camera.serial for camera in example.cameras}.isdisjoint(
        {camera.serial for camera in reference.cameras}
    )


def test_stream_and_recording_constraints_are_exact():
    cfg = load_config(CONFIG)

    assert (cfg.width, cfg.height, cfg.fps) == (1280, 720, 30)
    assert (cfg.color_format, cfg.color_encoding) == ("RGB8", "rgb8")
    assert (cfg.depth_format, cfg.depth_encoding) == ("Z16", "16UC1")
    assert cfg.max_bag_duration_seconds == 300
    assert cfg.max_bag_size_bytes == 137438953472
    assert cfg.max_cache_size_bytes == 1073741824
    assert cfg.additional_streams == ()
    assert tuple(field.name for field in fields(cfg)) == (
        "cameras",
        "width",
        "height",
        "fps",
        "color_format",
        "color_encoding",
        "depth_format",
        "depth_encoding",
        "max_bag_duration_seconds",
        "max_bag_size_bytes",
        "max_cache_size_bytes",
        "additional_streams",
    )


def test_configuration_dataclasses_are_frozen():
    cfg = load_config(CONFIG)

    with pytest.raises(FrozenInstanceError):
        cfg.fps = 29
    with pytest.raises(FrozenInstanceError):
        cfg.cameras[0].serial = "different"


def test_additional_glove_streams_are_validated_and_sorted(tmp_path):
    document = yaml.safe_load(CONFIG.read_text())
    document["recording"]["additional_streams"] = [
        {"topic": "/gloves/right/state", "type": "glove_msgs/msg/GloveState"},
        {"topic": "/gloves/left/state", "type": "glove_msgs/msg/GloveState"},
    ]
    cfg = load_config(_write_document(tmp_path, document))
    assert [stream.topic for stream in cfg.additional_streams] == [
        "/gloves/left/state",
        "/gloves/right/state",
    ]


@pytest.mark.parametrize(
    ("topic", "type_name"),
    [
        ("gloves/left/state", "glove_msgs/msg/GloveState"),
        ("/gloves/*", "glove_msgs/msg/GloveState"),
        ("/gloves/left/state", "GloveState"),
    ],
)
def test_invalid_additional_stream_is_rejected(tmp_path, topic, type_name):
    document = yaml.safe_load(CONFIG.read_text())
    document["recording"]["additional_streams"] = [
        {"topic": topic, "type": type_name}
    ]
    with pytest.raises(ValueError, match="absolute ROS topic|ROS message type"):
        load_config(_write_document(tmp_path, document))


def test_duplicate_serial_is_rejected(tmp_path):
    path = _write_modified_config(
        tmp_path, 'serial: "408322071716"', 'serial: "260322278433"'
    )

    with pytest.raises(ValueError, match="duplicate serial"):
        load_config(path)


def test_duplicate_asic_serial_is_rejected(tmp_path):
    document = yaml.safe_load(EXAMPLE_CONFIG.read_text())
    document["cameras"][1]["asic_serial"] = document["cameras"][0]["asic_serial"]
    path = _write_document(tmp_path, document)

    with pytest.raises(ValueError, match="duplicate asic_serial"):
        load_config(path)


def test_duplicate_name_is_rejected(tmp_path):
    path = _write_modified_config(tmp_path, "name: d436", "name: d405_1")

    with pytest.raises(ValueError, match="duplicate name"):
        load_config(path)


@pytest.mark.parametrize("name", ["", "405_front", "d405-front", "/d405_1", "d405 1"])
def test_camera_name_must_be_a_ros_safe_token(tmp_path, name):
    document = yaml.safe_load(EXAMPLE_CONFIG.read_text())
    document["cameras"][0]["name"] = name
    path = _write_document(tmp_path, document)

    with pytest.raises(ValueError, match="ROS name token"):
        load_config(path)


def test_non_30_hz_profile_is_rejected(tmp_path):
    path = _write_modified_config(tmp_path, "fps: 30", "fps: 29")

    with pytest.raises(ValueError, match="30 Hz"):
        load_config(path)


def test_unknown_color_module_is_rejected(tmp_path):
    path = _write_modified_config(
        tmp_path, "color_module: rgb_camera", "color_module: invented_camera"
    )

    with pytest.raises(ValueError, match="unknown color_module"):
        load_config(path)


def test_known_model_with_wrong_color_module_is_rejected(tmp_path):
    path = _write_modified_config(
        tmp_path, "color_module: depth_module", "color_module: rgb_camera"
    )

    with pytest.raises(ValueError, match="D405.*depth_module"):
        load_config(path)


@pytest.mark.parametrize(("parts", "replacement"), LOCKED_FIELD_MUTATIONS)
def test_fixed_topology_stream_and_recording_fields_reject_mutation(
    tmp_path, parts, replacement
):
    document = yaml.safe_load(CONFIG.read_text())
    _set_path(document, parts, replacement)
    path = _write_document(tmp_path, document)

    with pytest.raises(
        ValueError,
        match="camera topology|30 Hz|require color_module|fixed capture contract",
    ):
        load_config(path)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("empty", "exactly three"),
        ("extra", "exactly three"),
        ("reordered", "camera topology"),
    ],
)
def test_inventory_shape_and_topology_are_locked(tmp_path, mutation, message):
    document = yaml.safe_load(CONFIG.read_text())
    if mutation == "empty":
        document["cameras"] = []
    elif mutation == "extra":
        extra_camera = dict(document["cameras"][0])
        extra_camera.update(
            name="d405_3", serial="260322278435", asic_serial="255323071627"
        )
        document["cameras"].append(extra_camera)
    else:
        document["cameras"].reverse()
    path = _write_document(tmp_path, document)

    with pytest.raises(ValueError, match=message):
        load_config(path)


@pytest.mark.parametrize(
    ("section", "extra_key"),
    [
        pytest.param("root", "unapproved", id="root"),
        pytest.param("camera", "usb_port_id", id="camera"),
        pytest.param("stream", "profile_name", id="stream"),
        pytest.param("recording", "warmup_seconds", id="recording-warmup"),
        pytest.param(
            "recording", "stream_timeout_seconds", id="recording-timeout"
        ),
        pytest.param("recording", "min_fps", id="recording-min-fps"),
        pytest.param("recording", "max_fps", id="recording-max-fps"),
        pytest.param(
            "recording", "min_write_mb_s", id="recording-min-write"
        ),
        pytest.param(
            "recording", "capacity_margin", id="recording-capacity-margin"
        ),
        pytest.param(
            "recording", "disk_safety_bytes", id="recording-disk-safety"
        ),
    ],
)
def test_unexpected_configuration_keys_are_rejected(tmp_path, section, extra_key):
    document = yaml.safe_load(CONFIG.read_text())
    if section == "root":
        target = document
    elif section == "camera":
        target = document["cameras"][0]
    else:
        target = document[section]
    target[extra_key] = "unexpected"
    path = _write_document(tmp_path, document)

    with pytest.raises(ValueError, match="unexpected.*key"):
        load_config(path)


@pytest.mark.parametrize("non_finite", NON_FINITE_VALUES)
@pytest.mark.parametrize("parts", NUMERIC_FIELD_PATHS)
def test_non_finite_numeric_values_are_rejected(tmp_path, parts, non_finite):
    document = yaml.safe_load(CONFIG.read_text())
    _set_path(document, parts, non_finite)
    path = _write_document(tmp_path, document)

    with pytest.raises(ValueError, match="must be finite"):
        load_config(path)
