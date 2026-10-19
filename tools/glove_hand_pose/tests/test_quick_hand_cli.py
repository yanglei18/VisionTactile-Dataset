"""Fast-hand CLI dispatch is explicit and never opens a serial device itself."""

import sys
import types

from vt_glove_pose.cli import main, parser


def test_quick_hand_cli_dispatch(monkeypatch):
    calls = []
    module = types.ModuleType("vt_glove_pose.quick_hand_live")

    def launch(config, **kwargs):
        calls.append((config, kwargs))
        return 0

    module.launch_quick_hand = launch
    monkeypatch.setitem(sys.modules, "vt_glove_pose.quick_hand_live", module)
    assert main(["quick-live", "--port", "/dev/explicit-test", "--smoke-seconds", "0.1"]) == 0
    config, options = calls[0]
    assert config["hand"] == "right"
    assert options == {"port": "/dev/explicit-test", "smoke_seconds": 0.1}


def test_quick_hand_default_has_no_automatic_connection():
    args = parser().parse_args(["quick-live"])
    assert args.port == ""
    assert args.smoke_seconds is None
