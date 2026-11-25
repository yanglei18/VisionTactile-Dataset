import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from vt_capture_manager.optical.mount_model import load_mount_model


@pytest.mark.parametrize("hand", ["left", "right"])
def test_model_uses_manifest_checked_joints_and_glove_state_identity(hand, tmp_path, monkeypatch):
    # The standalone viewer adds the selected checkout's hand adapter just as its launcher does.
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[4] / "tools/fsglove12/src"))
    from vt_fsglove12.config import PARENTS, digest

    points = np.arange(63, dtype=float).reshape(21, 3) * .001
    provenance = {"hand": hand, "geometry_units": "metres"}
    rest = {"parents": list(PARENTS), "rest_global_xyzw": [[0., 0., 0., 1.]] * 16,
            "provenance": provenance}
    display = {"schema_version": 1, "model_axes": {"hand": hand},
               "neutral_geometry": {"provenance": provenance, "joints_m": points.tolist()}}
    docs = {"hand.json": display, "config.json": {"hand": hand}, "rest.json": rest,
            "tactile_layout.json": {}, f"mano/models/MANO_{hand.upper()}.pkl": "opaque model"}
    checksums = {}
    for filename, document in docs.items():
        path = tmp_path / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        raw = json.dumps(document).encode()
        path.write_bytes(raw)
        checksums[filename] = hashlib.sha256(raw).hexdigest()
    manifest = {"schema_version": 1, "product": "hand-motion-touch", "files": checksums}
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    model = load_mount_model(tmp_path, hand)
    np.testing.assert_allclose(model.joints_m, points - points[0])
    assert model.rest_sha256 == digest(rest)
    with pytest.raises(ValueError, match="资源未通过校验"):
        load_mount_model(tmp_path, "left" if hand == "right" else "right")
    (tmp_path / "hand.json").write_bytes(b"{}")
    with pytest.raises(ValueError, match="资源未通过校验"):
        load_mount_model(tmp_path, hand)


def test_missing_model_has_operator_instruction():
    with pytest.raises(ValueError, match="为左手选择动捕模型"):
        load_mount_model("", "left")
