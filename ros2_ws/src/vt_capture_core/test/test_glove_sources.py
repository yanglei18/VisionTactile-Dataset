from array import array
from types import SimpleNamespace

import pytest

from vt_capture_core.glove_sources import glove_sources


def sources(reference=11):
    return dict(reference_node_id=reference,
        segment_observed_mask=[i not in (2, 5, 8, 11, 13) for i in range(16)],
        joint_observed_mask=[i in (0, 1, 4, 7, 10, 15) for i in range(16)],
        inferred_mask=[i not in (0, 1, 4, 7, 10, 15) for i in range(16)])


@pytest.mark.parametrize("reference", [1, 11, 12])
def test_sources_preserve_sender_reference_and_masks_without_local_layout(reference):
    value = sources(reference)
    result = glove_sources(SimpleNamespace(**value))
    assert result == value
    result["segment_observed_mask"][0] = False
    assert value["segment_observed_mask"][0]
    assert glove_sources(value) == value


def test_ros_byte_boolean_arrays_are_supported():
    value = sources()
    for key in ("segment_observed_mask", "joint_observed_mask", "inferred_mask"):
        value[key] = array("b", value[key])
    assert glove_sources(SimpleNamespace(**value)) == sources()


@pytest.mark.parametrize("malformed", [None, [], [True] * 15, [1.] * 16, [2] * 16, "0" * 16])
def test_malformed_optional_metadata_does_not_invent_sources(malformed):
    value = sources()
    value["inferred_mask"] = malformed
    assert glove_sources(value) == {}


def test_missing_unknown_or_contradictory_sources_remain_unknown():
    assert glove_sources({}) == {}
    value = sources()
    value["inferred_mask"][0] = True
    assert glove_sources(value) == {}
    value = {key: [False] * 16 for key in sources() if key.endswith("mask")}
    assert glove_sources(value) == {}
