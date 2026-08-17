from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    RoleSnapshot,
    VisualHealth,
)

from vt_vive_tracker_gui.snapshot_store import LatestSnapshotStore


class FakeClock:
    def __init__(self, *values):
        self.values = list(values or (0,))

    def __call__(self):
        if len(self.values) > 1:
            return self.values.pop(0)
        return self.values[0]


class FakeRoot:
    def __init__(self):
        self.callbacks = {}
        self.cancelled = []
        self.destroy_count = 0
        self.last_delay_ms = None
        self._next_id = 0

    def after(self, delay_ms, callback):
        self._next_id += 1
        after_id = f"after-{self._next_id}"
        self.callbacks[after_id] = callback
        self.last_delay_ms = delay_ms
        return after_id

    def after_cancel(self, after_id):
        self.cancelled.append(after_id)
        self.callbacks.pop(after_id, None)

    def destroy(self):
        self.destroy_count += 1

    def run_next_callback(self):
        after_id = next(iter(self.callbacks))
        callback = self.callbacks.pop(after_id)
        callback()


class FakeView:
    def __init__(self, *, render_errors=()):
        self.rendered = []
        self.diagnostics = []
        self.render_errors = list(render_errors)

    def render(self, stored, fps):
        self.rendered.append((stored, fps))
        if self.render_errors:
            raise self.render_errors.pop(0)

    def set_diagnostic(self, text):
        self.diagnostics.append(text)


def snapshots(rate):
    return tuple(
        RoleSnapshot(role, VisualHealth.OFFLINE, None, (), rate, None)
        for role in FIXED_ROLES
    )


def application_class():
    from vt_vive_tracker_gui.app import TrackerApplication

    return TrackerApplication


def test_refresh_renders_latest_only_and_schedules_sixty_fps():
    root = FakeRoot()
    view = FakeView()
    store = LatestSnapshotStore()
    first = store.publish(snapshots(1.0))
    latest = store.publish(snapshots(2.0))
    app = application_class()(
        root,
        store,
        view,
        shutdown=lambda: None,
        monotonic_ns=FakeClock(1_000_000_000),
    )

    app.start()
    root.run_next_callback()

    assert view.rendered[-1][0] == latest
    assert view.rendered[-1][0] != first
    assert 0 <= root.last_delay_ms <= 17


def test_close_is_idempotent_and_only_shuts_down_owned_runtime():
    calls = []
    root = FakeRoot()
    app = application_class()(
        root,
        LatestSnapshotStore(),
        FakeView(),
        shutdown=lambda: calls.append("shutdown"),
        monotonic_ns=FakeClock(0),
    )
    app.start()

    app.close()
    app.close()

    assert calls == ["shutdown"]
    assert root.destroy_count == 1
    assert len(root.cancelled) == 1


def test_refresh_forwards_each_diagnostic_version_exactly_once():
    root = FakeRoot()
    view = FakeView()
    store = LatestSnapshotStore()
    store.publish_diagnostic("non-finite pose rejected", 10)
    app = application_class()(
        root,
        store,
        view,
        shutdown=lambda: None,
        monotonic_ns=FakeClock(0, 16_666_666),
    )
    app.start()

    root.run_next_callback()
    root.run_next_callback()

    assert view.diagnostics == ["non-finite pose rejected"]
    assert len(view.rendered) == 2


def test_one_frame_render_error_is_non_modal_and_keeps_refreshing():
    shutdown_calls = []
    root = FakeRoot()
    view = FakeView(render_errors=(RuntimeError("canvas failed"),))
    app = application_class()(
        root,
        LatestSnapshotStore(),
        view,
        shutdown=lambda: shutdown_calls.append("shutdown"),
        monotonic_ns=FakeClock(0, 16_666_666),
    )
    app.start()

    root.run_next_callback()
    assert shutdown_calls == []
    assert len(root.callbacks) == 1
    assert view.diagnostics and "canvas failed" in view.diagnostics[-1]

    root.run_next_callback()
    assert len(view.rendered) == 2
    assert shutdown_calls == []


def test_fps_counts_render_attempts_in_trailing_one_second_window():
    root = FakeRoot()
    view = FakeView()
    clock = FakeClock(0, 0, 500_000_000, 1_100_000_000)
    app = application_class()(
        root,
        LatestSnapshotStore(),
        view,
        shutdown=lambda: None,
        monotonic_ns=clock,
    )
    app.start()

    root.run_next_callback()
    root.run_next_callback()
    root.run_next_callback()

    assert [fps for _, fps in view.rendered] == [1.0, 2.0, 2.0]
