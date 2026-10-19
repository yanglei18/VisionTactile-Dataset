import hashlib
import json

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_joint_experiment import compare_export
from vt_glove_pose import hinge_joint_experiment as experiment


def fake_export(tmp_path):
    attempts, files = [], {}
    for i, kind in enumerate(("neutral", "motion"), 1):
        name = f"{i:03d}_{kind}.csv"
        data = b"raw file preserved for checksum test\n"
        (tmp_path/name).write_bytes(data)
        files[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
        attempts.append(dict(kind=kind, status="completed", connection=1,
                             matched_pairs=300, raw_file=name))
    manifest = dict(export_complete=True, config=load_config(), session=dict(proximal=5,distal=6),
                    attempts=attempts, files=files)
    (tmp_path/"manifest.json").write_text(json.dumps(manifest))
    return manifest


def test_explicit_units_and_rejected_fit_are_reported_without_installation(tmp_path, monkeypatch):
    fake_export(tmp_path)
    config=load_config(); config["gyro_dps_per_lsb_by_node"]={"5": .0175,"6": .0175}
    seen=[]
    def capture(path, **kwargs):
        seen.append(kwargs)
        return {"prox_gyro_dps": np.zeros((300,3)), "dist_gyro_dps": np.zeros((300,3)),
                "prox_acc_raw": np.tile([0,0,8197],(300,1)),
                "dist_acc_raw": np.tile([0,0,8197],(300,1))}
    monkeypatch.setattr(experiment,"load_pair_capture",capture)
    monkeypatch.setattr(experiment,"calibrate_hinge_pair",lambda *a,**kw: dict(
        profile_status="axis_inconsistent", neutral=dict(prox_bias_dps=[0,0,0],dist_bias_dps=[0,0,0])))
    monkeypatch.setattr(experiment,"fit_joint_hinge_axes",lambda *a,**kw: dict(
        candidate_status="inconsistent",axis_prox=None,axis_dist=None))
    before={p.name:p.read_bytes() for p in tmp_path.iterdir()}
    report=compare_export(tmp_path, config)
    assert seen[0]["gyro_dps_per_lsb_by_node"]=={"5":.0175,"6":.0175}
    assert report["recorded_config"].get("gyro_dps_per_lsb_by_node") is None
    assert report["evaluation_config"]["gyro_dps_per_lsb_by_node"]=={"5":.0175,"6":.0175}
    assert report["runs"][0]["joint_candidate"]["candidate_status"]=="inconsistent"
    assert report["live_calibration_changed"] is False
    assert before=={p.name:p.read_bytes() for p in tmp_path.iterdir()}


@pytest.mark.parametrize("damage",["hash","size","incomplete"])
def test_corrupted_export_is_rejected_before_fitting(tmp_path,damage):
    manifest=fake_export(tmp_path)
    if damage=="hash":
        manifest["files"]["001_neutral.csv"]["sha256"]="0"*64
    elif damage=="size":
        manifest["files"]["001_neutral.csv"]["bytes"]+=1
    else:
        manifest["export_complete"]=False
    (tmp_path/"manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        compare_export(tmp_path)


def test_motion_cannot_borrow_neutral_from_different_connection(tmp_path):
    manifest=fake_export(tmp_path)
    manifest["attempts"][1]["connection"]=2
    (tmp_path/"manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match="same connection"):
        compare_export(tmp_path)
