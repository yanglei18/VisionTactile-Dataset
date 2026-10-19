#!/usr/bin/env python3
"""Refresh already-authorized serial groups before starting the desktop tree.

``sg`` forks, so it must never wrap the manager process: the desktop authenticates
that process by its original PID.  Only the outer desktop launcher may refresh
its groups.  No device is opened and no account or device permission is changed.
"""

from __future__ import annotations

import argparse
import grp
import json
import os
from pathlib import Path
import pwd
import selectors
import shlex
import signal
import subprocess
import sys


GROUP = "dialout"
REFRESH_TIMEOUT_S = 8.0


def session_status() -> dict:
    """Describe account authorization separately from this process's groups."""
    result = dict(group=GROUP, gid=None, username="", account_has_group=False,
                  process_has_group=False, needs_refresh=False, detail="")
    try:
        account = pwd.getpwuid(os.getuid())
        result["username"] = account.pw_name
        group = grp.getgrnam(GROUP)
        result["gid"] = group.gr_gid
        result["process_has_group"] = group.gr_gid in {*os.getgroups(), os.getegid()}
        result["account_has_group"] = group.gr_gid in os.getgrouplist(account.pw_name, account.pw_gid)
    except (KeyError, OSError) as exc:
        result["detail"] = (
            "无法确认 dialout 串口组授权；相机、Tracker 和离线功能仍可使用。"
            "如需串口设备，请联系管理员检查账号组与设备权限。"
            f"（{exc}）"
        )
        return result
    result["needs_refresh"] = result["account_has_group"] and not result["process_has_group"]
    if result["needs_refresh"]:
        result["detail"] = (
            "账号已加入 dialout，但当前桌面进程尚未继承串口权限。"
            "请关闭并从应用菜单重新打开 Vision Tactile 采集工作台，"
            "或运行 bash tools/capture_deploy/start_workbench.sh；"
            "如果仍失败，请注销桌面会话后重新登录。不会修改设备权限或以 root 运行工作台。"
        )
    elif not result["process_has_group"]:
        result["detail"] = (
            "当前账号尚未获得 dialout 串口组权限；相机、Tracker 和离线功能仍可使用。"
            "如需串口设备，请联系管理员授权并重新登录。工作台不会索取密码或自动修改账号权限。"
        )
    else:
        result["detail"] = "当前进程已继承 dialout 串口组权限。"
    return result


def require_manager_session() -> dict:
    """Fail before a managed launch if an old desktop session lost authorization.

    Accounts without dialout remain usable for non-serial devices; serial-device
    preflight is responsible for reporting their missing device access.
    """
    status = session_status()
    if status["needs_refresh"]:
        raise RuntimeError(status["detail"])
    return status


def _exec_command(argv: list[str]) -> None:
    if not argv:
        raise RuntimeError("缺少工作台启动命令，请通过 start_workbench.sh 打开工作台。")
    try:
        os.execvp(argv[0], argv)
    except OSError as exc:
        raise RuntimeError(f"工作台启动失败：{exc}") from exc


def _refreshed(ready_fd: int, primary_gid: int, argv: list[str]) -> None:
    # A separate subcommand, not an environment marker, prevents re-exec loops.
    if not session_status()["process_has_group"]:
        raise RuntimeError("串口组会话刷新未生效；请注销桌面会话后重新登录，再打开工作台。")
    if os.getgid() != primary_gid or os.getegid() != primary_gid:
        raise RuntimeError("工作台主组恢复失败；请注销桌面会话后重新登录，再打开工作台。")
    os.write(ready_fd, b"ready\n")
    os.close(ready_fd)
    _exec_command(argv)


def _signal_refresh(process: subprocess.Popen, signum: int) -> None:
    # Only the new session created by this exact Popen call is ever signalled.
    if process.poll() is None:
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass


def _stop_refresh(process: subprocess.Popen) -> None:
    _signal_refresh(process, signal.SIGTERM)
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired:
        _signal_refresh(process, signal.SIGKILL)
        process.wait(timeout=1)


def _run_refreshed(argv: list[str], *, sg_path: str = "/usr/bin/sg",
                   timeout: float = REFRESH_TIMEOUT_S) -> int:
    """Bound only group authentication, not the interactive desktop lifetime."""
    primary_gid = os.getegid()
    try:
        primary_group = grp.getgrgid(primary_gid).gr_name
    except (KeyError, OSError) as exc:
        raise RuntimeError("无法确认工作台的原主组；请注销桌面会话后重新登录，再打开工作台。") from exc
    read_fd, write_fd = os.pipe()
    process = None
    handlers = {}
    pending_signals = []
    def forward(number, _frame):
        if process is None:
            pending_signals.append(number)
        else:
            _signal_refresh(process, number)
    try:
        # Install before Popen: a fast child can acknowledge readiness before
        # the parent receives the Popen object. Retain signals across that handoff.
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, forward)
        command = shlex.join([
            "exec", sys.executable, "-I", str(Path(__file__).resolve()),
            "_refreshed", "--ready-fd", str(write_fd), "--primary-gid", str(primary_gid), "--", *argv,
        ])
        # sg makes its selected group primary. Restore the launcher's original
        # primary group in a second hop while retaining dialout as supplementary.
        # Otherwise same-user desktop portals cannot inspect /proc/<GUI>/root
        # because Linux's credential checks also compare the primary GIDs.
        # Both hops share the same noninteractive, bounded readiness handshake.
        command = shlex.join(["exec", sg_path, primary_group, "-c", command])
        try:
            process = subprocess.Popen(
                [sg_path, GROUP, "-c", command], stdin=subprocess.DEVNULL,
                pass_fds=(write_fd,), start_new_session=True,
            )
        except OSError as exc:
            raise RuntimeError(
                f"无法刷新已授权的串口组：{exc}。请注销桌面会话后重新登录，再打开工作台。"
            ) from exc
        finally:
            os.close(write_fd)
            write_fd = -1
        for signum in pending_signals:
            _signal_refresh(process, signum)
        # sg cannot read a password from stdin or a controlling terminal.  The
        # child acknowledges successful authorization before it starts the GUI.
        with selectors.DefaultSelector() as selector:
            selector.register(read_fd, selectors.EVENT_READ)
            ready = selector.select(timeout)
            if not ready or os.read(read_fd, 32) != b"ready\n":
                _stop_refresh(process)
                raise RuntimeError(
                    "串口组会话刷新失败或超时；未继续启动工作台。"
                    "请注销桌面会话后重新登录，或联系管理员检查 dialout 授权和 sg 命令。"
                    "工作台不会索取密码或修改设备权限。"
                )
        returncode = process.wait()
        return returncode if returncode >= 0 else 128 - returncode
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)
        for signum, handler in handlers.items():
            signal.signal(signum, handler)


def launch_workbench(argv: list[str]) -> int:
    if not argv:
        raise RuntimeError("缺少工作台启动命令，请通过 start_workbench.sh 打开工作台。")
    status = session_status()
    if status["needs_refresh"]:
        print("正在为工作台刷新账号已有的 dialout 串口组权限……", file=sys.stderr, flush=True)
        return _run_refreshed(argv)
    # Missing authorization must not disable camera-only or offline workflows.
    _exec_command(argv)
    return 0  # os.execvp does not normally return.


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    subparsers.add_parser("check", help="报告账号授权与当前进程组，不改动系统")
    subparsers.add_parser("require", help="拒绝从未继承已有串口权限的旧会话启动受管后台")
    launch = subparsers.add_parser("launch", help="在需要时刷新桌面进程树的已有组权限")
    launch.add_argument("command", nargs=argparse.REMAINDER)
    refreshed = subparsers.add_parser("_refreshed")
    refreshed.add_argument("--ready-fd", required=True, type=int)
    refreshed.add_argument("--primary-gid", required=True, type=int)
    refreshed.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    try:
        if args.mode == "check":
            print(json.dumps(session_status(), ensure_ascii=False))
        elif args.mode == "require":
            require_manager_session()
        else:
            command = args.command[1:] if args.command[:1] == ["--"] else args.command
            if args.mode == "_refreshed":
                _refreshed(args.ready_fd, args.primary_gid, command)
            else:
                return launch_workbench(command)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
