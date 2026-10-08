"""Real spawn/pickle use of the trusted publication path, never hardware."""
from test_live_process import Runtime, controller, wait_for


class PublicationRuntime(Runtime):
    def _latest_pose_publication(self):
        value = super().latest_snapshot()
        value["runtime"]["publication_path"] = "trusted_child_only"
        return value

    def latest_pose_snapshot(self):
        raise AssertionError("child must use the trusted publication reader")


def test_spawn_uses_publication_reader_but_parent_still_owns_its_copy(controller):
    value = controller(runtime_factory=PublicationRuntime).start()
    initial = wait_for(lambda: (snapshot if (snapshot := value.latest_snapshot())["runtime"].get(
        "publication_path") else None))
    assert initial["runtime"]["publication_path"] == "trusted_child_only"
    assert not initial["runtime"]["connected"]
    assert value.submit("connect", port="FAKE")
    live = wait_for(lambda: (snapshot if (snapshot := value.latest_snapshot())["pose"]["pose_available"]
                            else None))
    live["geometry"]["vertices_m"].clear()
    live["runtime"]["publication_path"] = "parent mutation"
    following = value.latest_snapshot()
    assert following["geometry"]["vertices_m"]
    assert following["runtime"]["publication_path"] == "trusted_child_only"
    value.stop()
    assert value.join(3.)
