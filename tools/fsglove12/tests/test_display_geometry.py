"""Preview overload cannot create a queue of measurement work."""
import multiprocessing as mp
import time

from vt_fsglove12.display_geometry import GeometryPreviewWorker, _LatestSlot


class SlowRenderer:
    def __init__(self, *_args):
        pass

    def render_mano(self, pose, shape):
        time.sleep(.12)
        return {'joints_m': pose, 'provenance': {'betas': shape}}


def test_latest_mailbox_is_bounded_and_lock_never_blocks():
    slot = _LatestSlot(mp.get_context('spawn'), 128)
    assert slot.publish({'frame': 1})
    assert slot.publish({'frame': 2})
    revision, result = slot.take(0)
    assert revision == 2 and result == {'frame': 2}
    slot.lock.acquire()
    try:
        assert not slot.publish({'frame': 3})
        assert slot.take(0) == (0, None)
    finally:
        slot.lock.release()
    assert slot.take(revision) == (revision, None)


def test_slow_surface_replaces_previews_without_blocking_submit():
    worker = GeometryPreviewWorker('', 'left', renderer_factory=SlowRenderer)
    try:
        durations = []
        for frame in range(50):
            started = time.perf_counter()
            worker.submit(dict(frame=frame, pose={'mano_pose_rotvec': [frame], 'shape_params': [2]}))
            durations.append(time.perf_counter() - started)
            time.sleep(.002)
        deadline, results = time.monotonic() + 4, []
        while time.monotonic() < deadline:
            result = worker.poll()
            if result:
                results.append(result)
                if result['frame'] == 49:
                    break
            time.sleep(.01)
        assert results[-1]['frame'] == 49
        assert results[-1]['geometry']['joints_m'] == [49]
        assert results[-1]['superseded'] > 0
        assert worker.snapshot()['capacity'] == 1
        assert max(durations) < .10  # Deliberately much looser than normal IPC.
    finally:
        worker.close()
    assert not worker.snapshot()['alive']
