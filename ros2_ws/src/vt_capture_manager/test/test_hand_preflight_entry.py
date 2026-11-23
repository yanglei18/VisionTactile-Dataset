"""Candidate runtime policy reaches the manager without consuming ROS options."""
import pytest

from vt_capture_manager import node


@pytest.mark.parametrize("candidate_root", [None, "/candidate/checkout"])
def test_candidate_runtime_policy_is_explicit_and_ros_arguments_survive(monkeypatch, candidate_root):
    def run(args, ros_args):
        assert args.candidate_source_root == candidate_root
        assert args.enable_device_control
        assert ros_args == ["--ros-args", "--disable-stdout-logs"]
        return "manager"

    monkeypatch.setattr(node, "_run_manager", run)
    argv = ["--enable-device-control"]
    if candidate_root is not None:
        argv += ["--candidate-source-root", candidate_root]
    assert node.main([*argv, "--ros-args", "--disable-stdout-logs"]) == "manager"
