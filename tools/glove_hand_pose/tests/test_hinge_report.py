import json

import numpy as np
import pytest

from vt_glove_pose.hinge_report import render_comparison_html, _thin_peaks


def comparison():
    def replay(values):
        return dict(replay_status="completed", summary={"samples":len(values)},
                    series=[dict(nominal_time_s=i*.1,angle_deg=value) for i,value in enumerate(values)])
    return dict(kind="vt_glove_pose.offline_hinge_comparison",
                source={"source":'<script>alert("source")</script>'},
                baseline=replay([0.,20.,45.,20.,0.]),constrained=replay([0.,21.,44.,19.,0.]),
                disagreement={"rms_deg":.77})


def test_self_contained_real_comparison_is_not_labelled_ground_truth():
    html=render_comparison_html(comparison())
    assert "没有独立真实角度参考" in html
    assert "算法差值（不是真实角度误差）" in html
    assert '<script>' not in html and '<script src=' not in html
    assert '&lt;script&gt;' in html
    assert '<svg' in html and '<polyline' in html


def test_peak_preserving_plot_decimation_does_not_hide_spikes():
    signal=np.zeros(10000);signal[2345]=100.;signal[7812]=-40.
    points=_thin_peaks(signal,limit=100)
    assert len(points)<=100
    assert 2345 in points and 7812 in points and 0 in points and 9999 in points


def test_failed_initialization_is_reported_instead_of_fake_zero_plot():
    report=comparison()
    report["constrained"]["replay_status"]="initial_gravity_unobservable"
    report["constrained"]["series"]=[]
    html=render_comparison_html(report)
    assert 'initial_gravity_unobservable' in html
    assert '<polyline' not in html


def test_corrupt_plot_values_rejected():
    report=comparison()
    report["baseline"]["series"][2]["angle_deg"]=float("nan")
    with pytest.raises(ValueError,match="finite"):
        render_comparison_html(report)


def test_cli_exclusive_html_does_not_overwrite_or_share_json_path(tmp_path,monkeypatch):
    from vt_glove_pose.cli import _comparison_output
    from argparse import Namespace
    json_path=tmp_path/'report.json';html_path=tmp_path/'report.html'
    args=Namespace(output=json_path,html=html_path)
    _comparison_output(comparison(),args)
    assert json.loads(json_path.read_text())["kind"]==comparison()["kind"]
    before=html_path.read_bytes()
    with pytest.raises(FileExistsError):
        _comparison_output(comparison(),args)
    assert html_path.read_bytes()==before
    with pytest.raises(ValueError,match="different"):
        _comparison_output(comparison(),Namespace(output=tmp_path/'same',html=tmp_path/'same'))
    assert not (tmp_path/'same').exists()
