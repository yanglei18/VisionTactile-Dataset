# 开源发布准备

本工具生成当前工作区的可发布源码快照，不暂存文件、不提交、不推送，也不改写现有 Git 历史。它与传感器运行和真机验收无关。

## 检查和导出

在仓库根目录使用 Python 3.10 及以上和 Git，无需 ROS、Qt 或硬件：

```bash
python3 tools/release/prepare_release.py check
python3 tools/release/prepare_release.py export --output /tmp/vision-tactile-source.tar.gz
```

需要保留本机检查报告时可加 `--report /tmp/vision-tactile-readiness.json`，
同样不覆盖已有文件。报告可能列出本机被排除的文件名，应先审阅再分享；
它不会自动进入源码包。

输出路径必须不存在，父目录须已存在。检查失败不会生成源码包，已有文件不会被覆盖。命令输出 JSON 报告；`public_source_check=PASS` 只代表源码快照检查通过，不代表现有 Git 历史安全、依赖许可证齐全或硬件验收通过。

检查读取当前工作文件，包括符合公开范围的未跟踪新增源码，并在临时 Git 仓库中运行既有 `tools/check_public_tree.py`。不修改原仓库索引或对象库。公开范围为根级项目说明及两个独立手套入口、`.github/`、`docs/`、`ros2_ws/src/`、`tools/`；这之外的内容不会被顺便打包。

以下内容不会进入源码包：

- Git 历史、虚拟环境、构建缓存、运行日志、本机 `.runtime/` 和 `.local/`。
- bag、MCAP、USB 抓包、真实角色映射与私有输入目录、许可模型和模型权重。
- 根目录的两张硬件照片，以及仅作参考的 `Window_reference/` 整个目录。
- PyVUT 子模块源码和本机扩展 PyVUT。子模块 URL 留在 `.gitmodules`，固定提交留在包内 `PUBLIC_SOURCE_MANIFEST.json`。

源码包内的 manifest 记录文件路径、大小和 SHA-256，用于交付核对；不包含机器路径、凭据或被排除文件的内容。文本的执行权限按脚本入口规范化，不改变原工作区权限。源码包仍需要按根 README 安装依赖，不能替代虚拟环境或 ROS 部署。

## Git 历史必须单独处理

报告中的 `history_path_check=BLOCKED` 表示本地可达历史存在禁止公开的路径。即使 `public_source_check=PASS`，也不要直接推送当前分支。删除文件、加 `.gitignore` 或打新标签，都不会删除旧提交中的内容。

当前准备流程选择先交付无历史源码包，不修改现有开发历史。后续发布需要明确选定新的干净仓库，或审查并执行旧仓库的历史清理；该选择及 GitHub 上传不由本工具自动执行。路径扫描和常见令牌格式检查并非完整秘密检测，也不能证明远端已删除历史；发布前仍需人工检查授权与私有输入边界。

若决定从源码包建立新的公开历史，在新的空目录解包，检查 manifest 后再建立仓库。普通 tar 包不能保存 Git 子模块条目；维护者应从 manifest 取出 `pyvut` 的固定提交并恢复 gitlink，而不是把本机 PyVUT 源码复制进去。以下步骤仅用于已选定的全新发布目录：

```bash
git init
git add .
PYVUT_COMMIT=$(python3 -c 'import json; print(json.load(open("PUBLIC_SOURCE_MANIFEST.json"))["submodules"]["pyvut"])')
git update-index --add --cacheinfo "160000,$PYVUT_COMMIT,pyvut"
python3 tools/check_public_tree.py
git diff --cached --check
```

这些步骤没有提交或推送命令。先完成[发布检查表](../../docs/release-checklist.md)，再由维护者选择提交、仓库和版本；不要用强推替代历史处理。

## 维护测试

```bash
python3 -m unittest discover -s tools/release/tests -v
python3 tools/test_check_public_tree.py
```

仅单元测试使用临时合成仓库，不连接设备，也不读取真实 Capture/Bundle。运行时软件测试与各硬件验收见 [CONTRIBUTING](../../CONTRIBUTING.md) 和[最终用户手册](../../docs/user-manual.md)。
