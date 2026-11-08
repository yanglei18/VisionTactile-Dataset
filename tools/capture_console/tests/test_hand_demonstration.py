"""Teaching geometry is validated before entering a live hand view."""
import sys
import threading
from copy import deepcopy

import numpy as np
import pytest

from vt_capture_console.backend.hand_demonstration import load_demonstration, validate_clips


def geometry():
    neutral = dict(vertices_m=np.zeros((4, 3)), joints_m=np.zeros((21, 3)))
    pair = [np.zeros((17, 4, 3)).tolist(), np.zeros((17, 21, 3)).tolist()]
    pair[0][-1][0][0] = .01
    return neutral, dict(schema_version=1, hand="left", synthetic_demonstration=True,
                         clips={name: deepcopy(pair) for name in ("flex_fingers", "flex_thumb")})


@pytest.mark.parametrize("invalid", ["hand", "live", "nonfinite", "neutral", "shape"])
def test_teaching_frames_cannot_be_confused_with_other_hand_or_unchecked_geometry(invalid):
    neutral, document = geometry()
    if invalid == "hand": document["hand"] = "right"
    elif invalid == "live": document["synthetic_demonstration"] = False
    elif invalid == "nonfinite": document["clips"]["flex_thumb"][0][-1][0][0] = float("nan")
    elif invalid == "neutral": document["clips"]["flex_fingers"][1][0][0][0] = .1
    elif invalid == "shape": document["clips"]["flex_thumb"][0].pop()
    with pytest.raises(ValueError):
        validate_clips(document, "left", neutral)


def test_timed_out_numerical_helper_is_reaped_and_never_uses_a_shell(monkeypatch):
    import subprocess
    from vt_capture_console.backend import hand_demonstration as module
    calls = []
    class Process:
        returncode = None
        def poll(self): return self.returncode
        def terminate(self): calls.append("terminate")
        def wait(self, timeout):
            calls.append(("wait", timeout))
            if self.returncode is None:
                raise subprocess.TimeoutExpired("owned-helper", timeout)
            return self.returncode
        def kill(self):
            calls.append("kill")
            self.returncode = -9
    def start(argv, **kwargs):
        assert isinstance(argv, list) and argv[1:4] == ["-I", "-m", "vt_fsglove12.teaching_geometry"]
        assert not kwargs.get("shell")
        assert kwargs["stdin"] == subprocess.DEVNULL
        return Process()
    monkeypatch.setattr(module.subprocess, "Popen", start)
    with pytest.raises(TimeoutError):
        load_demonstration(sys.executable, "/bundle", "left", {}, cancelled=threading.Event(), timeout=0)
    assert calls == ["terminate", ("wait", .5), "kill", ("wait", 1.)]
