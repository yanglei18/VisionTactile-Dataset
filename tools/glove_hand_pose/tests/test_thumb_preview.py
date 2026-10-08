import copy
import hashlib
import json

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import RawRecorder
from vt_glove_pose.thumb_preview import build_preview, replay_orientation


def make_export(directory):
    files, attempts = {}, []
    for number, kind, start, raw in ((1, "neutral", 10., 0), (2, "motion", 25., 2000)):
        name = f"{number:03d}_{kind}.csv"
        with RawRecorder(directory/name) as recorder:
            for tick in range(241):
                t = start+tick/120
                recorder.write([Sample(n, (0,0,raw,0,0,8197),t,1000+t) for n in range(1,7)])
        data = (directory/name).read_bytes()
        files[name] = dict(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
        attempts.append(dict(kind=kind, status="completed", connection=1,
                             matched_pairs=241,raw_file=name))
    manifest = dict(export_complete=True,config=load_config(),session=dict(proximal=5,distal=6),
                    attempts=attempts,files=files)
    (directory/"manifest.json").write_text(json.dumps(manifest))


def test_real_vqf_export_compares_explicit_units_without_reusing_missing_interval(tmp_path):
    make_export(tmp_path)
    before = {p.name:p.read_bytes() for p in tmp_path.iterdir()}
    config=load_config();config["gyro_dps_per_lsb_by_node"]={"5":.0175,"6":.0175}
    data=build_preview(tmp_path,config)
    run=data["runs"][0]
    assert run["summary"]["recorded_gap_not_integrated"]
    assert run["summary"]["neutral_used_for"]=="gyro_bias_only"
    assert run["samples"][0]["rotation_prox_deg"]==pytest.approx(0.)
    assert run["samples"][-1]["rotation_prox_deg"]==pytest.approx(70.,abs=.1)
    assert run["legacy_samples"][-1]["rotation_prox_deg"]==pytest.approx(35.,abs=.1)
    assert run["samples"][-1]["time_s"]==pytest.approx(2.)
    assert not data["anatomical_joint_angle"]
    assert before=={p.name:p.read_bytes() for p in tmp_path.iterdir()}
    json.dumps(data,allow_nan=False)


def test_mismatched_raw_hash_is_rejected(tmp_path):
    make_export(tmp_path)
    path=tmp_path/"002_motion.csv"
    path.write_bytes(path.read_bytes().replace(b"2000",b"1000"))
    with pytest.raises(ValueError,match="hash mismatch"):
        build_preview(tmp_path,load_config())


def test_replay_accepts_proximal_node_larger_than_distal():
    n=5
    capture={"host_monotonic_s":np.tile(np.arange(n)[:,None]/120,(1,2)),
             "host_wall_s":np.tile(1000+np.arange(n)[:,None]/120,(1,2))}
    for label in ("prox","dist"):
        capture[f"{label}_gyro_raw"]=np.zeros((n,3),dtype=int)
        capture[f"{label}_gyro_dps"]=np.zeros((n,3))
        capture[f"{label}_acc_raw"]=np.tile([0,0,8197],(n,1))
    rows=replay_orientation(capture,capture,load_config(),[6,5])
    assert len(rows)==n
    assert rows[-1]["rotation_dist_deg"]==pytest.approx(0.)


class FakeSource:
    def __init__(self, port):
        self.port=port;self.opened=False;self.closed=False
    def open(self): self.opened=True;return self
    def close(self): self.closed=True


def test_live_worker_requires_explicit_connect_reference_and_discards_stale_data():
    from vt_glove_pose.thumb_preview_live import OrientationWorker
    now=[10.]
    worker=OrientationWorker(load_config(),source_factory=FakeSource,clock=lambda:now[0])
    assert worker.source is None
    worker.publish(now[0]);assert worker.latest_snapshot()["current"] is None
    worker.command("connect",dict(port="test-device"))
    source=worker.source
    assert source.opened
    for node in (5,6):worker.processor.process(Sample(node,(0,0,0,0,0,8197),10.,1010.))
    worker.publish(10.);assert worker.latest_snapshot()["current"] is None
    worker.command("reference",{})
    worker.publish(10.)
    assert worker.latest_snapshot()["current"]["rotation_prox_deg"]==pytest.approx(0.)
    now[0]=11.
    worker.publish(11.)
    assert worker.latest_snapshot()["current"] is None
    with pytest.raises(ValueError,match="新鲜"):
        worker.command("reference",{})
    worker.command("disconnect",{})
    assert source.closed and worker.source is None
    assert not worker.history


def test_failed_open_releases_partial_source():
    from vt_glove_pose.thumb_preview_live import OrientationWorker
    created=[]
    class FailingSource(FakeSource):
        def open(self):
            created.append(self)
            raise OSError("busy")
    worker=OrientationWorker(load_config(),source_factory=FailingSource)
    with pytest.raises(OSError,match="busy"):
        worker.command("connect",dict(port="busy-device"))
    assert created[0].closed and worker.source is None


def test_live_window_starts_unconnected_and_closes_cleanly():
    from PyQt5 import QtWidgets
    from vt_glove_pose.thumb_preview_live import LiveOrientationWindow,OrientationWorker
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config=load_config()
    worker=OrientationWorker(config,source_factory=FakeSource)
    window=LiveOrientationWindow(config,port="test-device",worker=worker)
    window.show();app.processEvents();window.refresh()
    assert worker.source is None
    assert window.port.currentText()=="test-device"
    assert not window.reference_button.isEnabled()
    window.close();app.processEvents()
    assert not worker.isRunning()
