"""Trial admission reads selected cached artifacts, never live service sockets."""
from copy import deepcopy
from threading import RLock

import pytest

from vt_capture_manager.hand_control import HandControl
from vt_capture_manager.management import ManagementService


@pytest.fixture
def cached():
    now = [10.]
    calls = []
    evidence = dict(schema_version=1, hand="right", status="verified_snapshot", service_instance_id="service",
        session_id="session", profile_version=1, config_sha256="c"*64, rest_sha256="r"*64,
        profile_sha256="p"*64, source_mode="live", source_indices={}, artifacts={},
        producer_observed_monotonic_ns=1000, boundary_exact=False)
    response = dict(hand="right", service_instance_id="service", service_state="ready", pose_valid=True,
        calibration_busy=False, receipts=[], snapshot=dict(runtime=dict(connected=True, session_id="session", last_sample_age_s=.01),
            calibration=dict(profile_version=1)), trial_evidence=evidence)
    fail = [False]
    def exchange(path, request, **kwargs):
        calls.append(request)
        if request["op"] == "trial_evidence":
            if fail[0]:
                raise OSError("temporary disk/service delay")
            value = deepcopy(evidence)
            value["artifacts"] = {name:dict(sha256="a"*64, document={"kind":name}) for name in ("config","rest","profile")}
            return dict(ok=True, hand="right", service_instance_id="service", evidence=value)
        return deepcopy(response)
    control = HandControl([dict(id="right_glove", preset="glove", enabled=True, parameters=dict(hand="right"))],
        manager_instance_id="manager", allowed=lambda: (True,""), admission_lock=RLock(),
        process_snapshot=lambda: dict(groups=[]), exchange=exchange, monotonic=lambda:now[0])
    return control, now, calls, evidence, response, fail


def test_cache_is_lazy_selected_and_admission_does_not_exchange(cached):
    control, now, calls, evidence, response, fail = cached
    assert control.trial_evidence("before_start", ("right",))["hands"]["right"]["status"] == "unavailable"
    assert not calls
    control._poll("right")
    assert [call["op"] for call in calls] == ["snapshot", "trial_evidence"]
    before = len(calls)
    selected = control.trial_evidence("before_start", ("right",))
    assert selected["hands"]["right"]["status"] == "verified_snapshot"
    assert set(selected["hands"]["right"]["artifacts"]) == {"config","rest","profile"}
    assert selected["hands"]["right"]["producer_observed_monotonic_ns"] == 1000
    assert control.trial_evidence("before_start", ())["hands"] == {}
    assert control.trial_evidence("after_stop", ("right",))["hands"]["right"]["artifacts"] == {}
    assert len(calls) == before
    control._poll("right")
    assert len(calls) == before + 1  # unchanged profile does not re-read files


def test_cache_failure_recovers_without_reconnect_and_new_profile_cannot_use_old_files(cached):
    control, now, calls, evidence, response, fail = cached
    fail[0] = True
    control._poll("right")
    assert control.trial_evidence("before_start", ("right",))["hands"]["right"]["status"] == "unavailable"
    fail[0] = False
    now[0] += 5.1
    control._poll("right")
    assert control.trial_evidence("before_start", ("right",))["hands"]["right"]["status"] == "verified_snapshot"
    evidence["profile_version"] = 2
    evidence["profile_sha256"] = "n"*64
    fail[0] = True
    control._poll("right")
    assert control.trial_evidence("before_start", ("right",))["hands"]["right"]["status"] == "unavailable"
    now[0] += 5.1
    fail[0] = False
    control._poll("right")
    assert control.trial_evidence("before_start", ("right",))["hands"]["right"]["profile_version"] == 2
    assert all(call["op"] in {"snapshot", "trial_evidence"} for call in calls)


def test_stale_cache_and_unmanaged_hand_remain_explicit(cached):
    control, now, calls, *_ = cached
    control._poll("right")
    now[0] += 2.
    result = control.trial_evidence("before_start", ("right", "left"))["hands"]
    assert result["right"]["status"] == result["left"]["status"] == "unavailable"
    assert "受管来源" in result["left"]["problems"][0]


def test_plan_selection_and_formal_evidence_fail_closed_without_affecting_nonhand():
    select = ManagementService._selected_pose_hands
    assert select(dict(recipe=dict(pose_hands=[])),dict(topics=["/gloves/right/state"])) == ()
    assert select(dict(recipe=dict(pose_hands=["left"])),{}) == ("left",)
    assert select({},dict(topics=["/gloves/right/state","/gloves/left/tactile/samples"])) == ("right",)
    problem = ManagementService._hand_evidence_problem
    assert problem((), None) == ""
    assert "不可追溯" in problem(("right",), None)
    row = dict(status="verified_snapshot", service_instance_id="service",session_id="s",profile_version=1,
               config_sha256="c",rest_sha256="r",profile_sha256="p",artifacts={name:{} for name in ("config","rest","profile")})
    evidence = dict(hands=dict(right=row))
    assert problem(("right",),evidence) == ""
    previous = deepcopy(evidence)
    row["profile_version"] = 2
    assert "排队期间改变" in problem(("right",), evidence, previous)
