"""Validate an explicitly prepared candidate before opening Qt or its backend."""
from pathlib import Path

from vt_capture_core.software_identity import runtime_identity

from .workstation_settings import private_directory, read_private_json, repository_root


def load_candidate_binding(directory, *, repo_root=None):
    root = Path(repo_root if repo_root is not None else repository_root()).resolve(strict=True)
    directory = Path(directory).expanduser().absolute()
    if not directory.is_dir():
        raise ValueError("候选配置尚未准备，请先运行配置迁移工具。")
    private_directory(directory)
    if directory.resolve() == (Path.home() / ".config/visiontactile/workbench").resolve():
        raise ValueError("候选程序不能复用全局工作台配置目录。")
    manifest = read_private_json(directory / "migration.json")
    if (manifest.get("schema_version") != 1 or manifest.get("kind") != "visiontactile_candidate_migration"
            or manifest.get("repo_root") != str(root)):
        raise ValueError("候选配置与当前安装的代码目录不匹配，请使用创建它的工作树入口。")
    configuration = directory / "workstation.json"
    read_private_json(configuration)
    if len(str(directory / "private-desktop.sock").encode()) > 107:
        raise ValueError("候选设置目录过长，无法创建本地后台连接。")
    return directory, runtime_identity(root, configuration)
