from vt_capture_console.viewmodels.recorder_metrics import recorder_readout


def test_recording_readout_uses_only_storage_measurements():
    snapshot = {"recording": {"state": "RECORDING"}, "devices": [{"receive_hz": 999}]}
    label, details = recorder_readout(snapshot)
    assert label and not details and "999" not in str(label)
    snapshot["recorder_metrics"] = dict(storage_mb_s=None, writer_errors=0, storage_errors=0,
        topics=[dict(topic="/gloves/left/imu/raw", received=100, written=70, written_hz=21.5)])
    label, details = recorder_readout(snapshot)
    assert "70" in str(label) and "999" not in details
    assert "/gloves/left/imu/raw" in details and "21.5 Hz" in details and "100" in details
    snapshot["recording"]["state"] = "IDLE"
    del snapshot["recorder_metrics"]
    assert recorder_readout(snapshot) == ("", "")
