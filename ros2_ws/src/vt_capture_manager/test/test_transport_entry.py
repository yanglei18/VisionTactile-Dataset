"""Configure DDS before importing ROS, so every managed child inherits it."""
import builtins
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_capture_manager import node


def test_manager_configures_transport_before_ros_import(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    for key in ("RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                "FASTDDS_BUILTIN_TRANSPORTS", "SKIP_DEFAULT_XML", "FASTDDS_ENVIRONMENT_FILE"):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(node, "load_config", lambda path: object())
    original_import = builtins.__import__

    class ReachedROS(Exception):
        pass

    def check_import(name, *args, **kwargs):
        if name == "rclpy":
            assert Path(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]).is_file()
            raise ReachedROS
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", check_import)
    with pytest.raises(ReachedROS):
        node._run_manager(SimpleNamespace(config=None), [])
