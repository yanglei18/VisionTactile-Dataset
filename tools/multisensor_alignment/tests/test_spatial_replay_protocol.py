"""The scene worker echoes request identity even for failed frame decoding."""
import io
import json
from types import SimpleNamespace

from vt_multisensor_alignment import spatial_replay_worker as worker


def test_frame_success_and_error_echo_request_id_and_index(monkeypatch):
    replies = []
    closed = []

    class Source:
        frame_count = 2
        note = "synthetic"
        last_clouds = []

        def frame(self, index):
            if index == 1:
                raise ValueError("bad frame")
            return {"sequence": index + 1}

        def close(self):
            closed.append(True)

    payload = b"\n".join(json.dumps(dict(command="frame", index=index, request_id=index + 11)).encode()
                          for index in (0, 1)) + b"\n"
    monkeypatch.setattr(worker, "BagReplaySource", lambda path: Source())
    monkeypatch.setattr(worker, "_reply", replies.append)
    monkeypatch.setattr(worker.sys, "stdin", SimpleNamespace(buffer=io.BytesIO(payload)))
    assert worker.main(["unused"]) == 0
    assert replies[1] == dict(ok=True, request_id=11, index=0, scene={"sequence": 1}, clouds=[])
    assert replies[2] == dict(ok=False, request_id=12, index=1, error="bad frame")
    assert closed == [True]
