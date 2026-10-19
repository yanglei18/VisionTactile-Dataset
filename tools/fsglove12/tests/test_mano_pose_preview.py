"""Display worker forwards solved MANO, including root, without another solve."""
from io import StringIO
import json

import numpy as np
import pytest

from vt_fsglove12.mano_pose_preview import render_request, serve, MAX_INPUT_CHARS


class Backend:
    def __init__(self):
        self.calls = []

    def render_mano(self, pose, shape):
        self.calls.append((np.array(pose), np.array(shape)))
        joints = np.zeros((21, 3))
        joints[5], joints[9], joints[17] = [-.08, 0, .03], [-.09, 0, 0], [-.07, 0, -.04]
        return dict(vertices_m=np.zeros((778, 3)).tolist(), joints_m=joints.tolist(), faces=[[0, 1, 2]])


def request(hand="right"):
    return dict(id=4, mano=dict(hand=hand, pose=[.1] * 48, shape=[.2] * 10))


def test_exact_pose_and_shape_forwarded_without_extra_root_rotation():
    backend = Backend()
    value = request()
    value["mano"]["pose"][:3] = [.3, -.4, .7]
    result = render_request(backend, value, "right")
    assert result["id"] == 4 and result["type"] == "geometry"
    assert set(result["geometry"]) == {"vertices_m", "joints_m"}
    np.testing.assert_array_equal(backend.calls[0][0], np.asarray(value["mano"]["pose"]).reshape(16, 3))
    np.testing.assert_array_equal(backend.calls[0][1], value["mano"]["shape"])


@pytest.mark.parametrize("change", [lambda v: v.update(id=True),
    lambda v: v["mano"].update(hand="left"), lambda v: v["mano"].update(pose=[0.] * 47),
    lambda v: v["mano"].update(shape=[float("nan")] * 10)])
def test_invalid_requests_never_render(change):
    backend, value = Backend(), request()
    change(value)
    with pytest.raises(ValueError):
        render_request(backend, value, "right")
    assert not backend.calls


def test_published_joint_check_rejects_wrong_model_but_allows_roundoff():
    backend = Backend()
    value = request()
    joints = np.asarray(backend.render_mano(np.zeros((16, 3)), np.zeros(10))["joints_m"])
    value["joints"] = (joints + [.001, 0, 0]).tolist()
    assert render_request(backend, value, "right")["type"] == "geometry"
    value["joints"] = (joints + [.003, 0, 0]).tolist()
    with pytest.raises(ValueError, match="does not match published joints"):
        render_request(backend, value, "right")


def test_persistent_worker_emits_ready_then_geometry_and_recovers_request_errors():
    values = [request(), request("left"), request()]
    sink = StringIO()
    serve(Backend(), "right", StringIO("".join(json.dumps(v) + "\n" for v in values)), sink)
    messages = [json.loads(line) for line in sink.getvalue().splitlines()]
    assert [v["type"] for v in messages] == ["ready", "geometry", "error", "geometry"]
    assert messages[0]["bundle"]["neutral_geometry"]["provenance"]["hand"] == "right"


def test_worker_bounds_input_size():
    with pytest.raises(ValueError, match="size limit"):
        serve(Backend(), "right", StringIO("x" * (MAX_INPUT_CHARS + 2)), StringIO())
