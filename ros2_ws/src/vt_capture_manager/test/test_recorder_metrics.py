from types import SimpleNamespace as NS

from vt_capture_core.recorder_metrics import STORAGE_WRITE_SEMANTICS
from vt_capture_manager.recorder_metrics import RecorderMeasurements


def message(stamp, count, instance="a"):
    return NS(schema_version=1, session_id="trial", recorder_instance_id=instance,
              observed_monotonic_ns=stamp, storage_write_semantics=STORAGE_WRITE_SEMANTICS,
              available=True, closed=False, rate_available=False, storage_returned_mb_s=0.,
              writer_errors=0, storage_errors=0, topics=[NS(topic="/test",
                   writer_received_messages=count+5, storage_returned_messages=count)])


def test_rates_belong_to_fresh_recording_and_recorder_process():
    view = RecorderMeasurements()
    view.observe(message(1_000_000_000, 40), 1_050_000_000)
    assert view.snapshot("trial", 1_050_000_000, 500_000_000)["topics"][0]["written_hz"] is None
    view.observe(message(1_250_000_000, 45), 1_300_000_000)
    view.observe(message(1_200_000_000, 40), 1_300_000_000)
    assert view.snapshot("trial", 1_300_000_000, 500_000_000)["topics"][0]["written_hz"] == 20.
    assert view.snapshot("other", 1_300_000_000, 500_000_000) is None
    assert view.snapshot("trial", 1_900_000_000, 500_000_000) is None
    view.observe(message(2_000_000_000, 2, "b"), 2_100_000_000)
    assert view.snapshot("trial", 2_100_000_000, 500_000_000)["topics"][0]["written_hz"] is None
    # A controller can explicitly withdraw an unreadable metrics snapshot;
    # this observation has no recorder stamp or storage semantics.
    view.observe(NS(schema_version=1, session_id="trial", available=False), 2_200_000_000)
    assert view.snapshot("trial", 2_200_000_000, 500_000_000) is None
