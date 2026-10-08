"""Read-only serial launch admission; never open, configure or reset a port.

Device processes inherit this process's effective credentials. Checking those
credentials (rather than only the account database) catches managers left alive
after a user was added to dialout. This is an advisory preflight, not a guarantee
against the device being unplugged or its permissions changing after admission.
"""
from __future__ import annotations

import grp
import os
import pwd


def _permission_detail(path, device_stat=None):
    detail = f"串口 {path} 权限不足：当前采集后台进程没有读写权限。"
    try:
        account = pwd.getpwuid(os.geteuid())
        process_groups = set(os.getgroups()) | {os.getegid()}
        account_groups = set(os.getgrouplist(account.pw_name, account.pw_gid))
        group_id = device_stat.st_gid if device_stat is not None else grp.getgrnam("dialout").gr_gid
        try:
            group_name = grp.getgrgid(group_id).gr_name
        except KeyError:
            group_name = str(group_id)
        if group_id in account_groups and group_id not in process_groups:
            return (detail + f"账户 {account.pw_name} 已获 {group_name} 组授权，但后台仍使用旧会话权限。"
                    "请刷新并重启采集后台；若仍无权限，请退出并重新登录桌面后重开采集界面。"
                    "无需重复添加用户组。")
        if group_id not in account_groups:
            return (detail + f"账户 {account.pw_name} 尚未获得串口所属 {group_name} 组的授权。"
                    "请由管理员完成项目的串口权限部署（用户组 / udev 规则），"
                    "然后重新登录并重启采集后台。")
        return (detail + f"账户及后台进程已包含 {group_name} 组，"
                "请检查设备节点读写权限、ACL 和 udev 规则，并重新部署串口权限。")
    except (KeyError, OSError):
        return detail + "无法完整读取账户权限信息；请检查串口权限部署，并重新登录后重启采集后台。"


def serial_start_problem(resources):
    """Return a receipt-compatible ``(code, Chinese detail)`` or ``None``.

    Only explicit ``serial:`` resources participate. ``stat`` and ``access`` do
    not acquire the device, so they cannot toggle modem-control lines or reset
    hardware. Account/group lookups occur only after an actual access failure.
    """
    for resource in resources:
        if not resource.startswith("serial:"):
            continue
        path = resource.removeprefix("serial:")
        if not path:
            return "serial_unavailable", "串口路径未配置；请检查设备启动配置。"
        try:
            device_stat = os.stat(path)
        except (FileNotFoundError, NotADirectoryError):
            return "serial_unavailable", f"串口 {path} 不存在；请检查 USB 连接、设备枚举和配置端口。"
        except PermissionError:
            return "serial_permission_denied", _permission_detail(path)
        except OSError as exc:
            return "serial_unavailable", f"无法检查串口 {path}：{exc}；设备未启动。"
        try:
            accessible = os.access(path, os.R_OK | os.W_OK, effective_ids=True)
        except NotImplementedError:
            return "serial_unavailable", f"无法检查串口 {path} 的有效身份读写权限；设备未启动。"
        except PermissionError:
            accessible = False
        except OSError as exc:
            return "serial_unavailable", f"无法检查串口 {path}：{exc}；设备未启动。"
        if not accessible:
            # A hot-unplug between stat and access must not be misreported as a
            # missing user-group grant. Do not open the port to investigate it.
            try:
                device_stat = os.stat(path)
            except (FileNotFoundError, NotADirectoryError):
                return "serial_unavailable", f"串口 {path} 已断开或不存在；请检查 USB 连接和配置端口。"
            except PermissionError:
                pass
            except OSError as exc:
                return "serial_unavailable", f"无法检查串口 {path}：{exc}；设备未启动。"
            return "serial_permission_denied", _permission_detail(path, device_stat)
    return None
