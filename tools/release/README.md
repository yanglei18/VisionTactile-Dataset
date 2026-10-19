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
- 本机保存的厂商标定板产品图片、Windows 安装包和其他编译产物。
- PyVUT 子模块源码和本机扩展 PyVUT。子模块 URL 留在 `.gitmodules`，固定提交留在包内 `PUBLIC_SOURCE_MANIFEST.json`。

静态手模型即使改名为普通 JSON，源码快照检查也会根据导出格式阻止发布。
测试使用运行时生成的合成几何，不随源码保存许可模型或其衍生网格。
Windows 安装器的上游语言和许可文件保留原始 CRLF 字节；该目录的局部
Git 属性允许这种换行，不关闭其他源码的空白检查，也不修改固定的上游哈希。

源码包内的 manifest 记录文件路径、大小和 SHA-256，用于交付核对；不包含机器路径、凭据或被排除文件的内容。文本的执行权限按脚本入口规范化，不改变原工作区权限。源码包仍需要按根 README 安装依赖，不能替代虚拟环境或 ROS 部署。

## Git 历史必须单独处理

报告中的 `history_path_check=BLOCKED` 表示本地可达历史存在禁止公开的路径。即使 `public_source_check=PASS`，也不要直接推送当前分支。删除文件、加 `.gitignore` 或打新标签，都不会删除旧提交中的内容。

当前准备流程选择先交付无历史源码包，不修改现有开发历史。后续发布需要明确选定新的干净仓库，或审查并执行旧仓库的历史清理；该选择及 GitHub 上传不由本工具自动执行。路径扫描和常见令牌格式检查并非完整秘密检测，也不能证明远端已删除历史；发布前仍需人工检查授权与私有输入边界。

若决定从源码包建立新的公开历史，在新的空目录解包，检查 manifest 后再建立仓库。普通 tar 包不能保存 Git 子模块条目；维护者应从 manifest 取出 `pyvut` 的固定提交并恢复 gitlink，而不是把本机 PyVUT 源码复制进去。以下步骤仅用于已选定的全新发布目录：

```bash
git init --initial-branch=main
git add .
PYVUT_COMMIT=$(python3 -c 'import json; print(json.load(open("PUBLIC_SOURCE_MANIFEST.json"))["submodules"]["pyvut"])')
git update-index --add --cacheinfo "160000,$PYVUT_COMMIT,pyvut"
python3 tools/check_public_tree.py
git diff --cached --check
git status --short
```

此目录拥有独立的 Git 索引和对象库，不要从开发目录复制 `.git`、添加旧仓库
为远端或合并旧分支。`CameraTrackerCalibration.msg`、Windows CI 等新增文件
应与其实现一起进入这个索引，避免只复制已有提交而遗漏未跟踪的源码。

这些步骤将完整候选暂存，供提交前审阅，尚未创建提交或推送。先完成
[发布检查表](../../docs/release-checklist.md)，再由维护者以自己的提交身份建立
首个提交、设置 GitHub 目标仓库和发布版本；不要用强推替代历史处理。
源码包的 SHA-256、公开文件清单和软件验证结果应与这份候选一起保留。
本机检查日志可能带机器路径，应留在交付目录之外，不整目录上传工作空间。

触觉工具的 `0.5.1` 是独立组件版本，不能据此把整套系统标记为 `v0.5.1`
正式版。当前候选按源码开发预览说明能力和验收范围。现成触觉 EXE 内含
单独授权的模型；公开源码准备通过，不代表该安装包可以作为公开 Release
附件。二进制发布需要另行核实模型及依赖许可和真实 Windows 硬件验收。

## 安装包验证

在准备好的源码候选目录中，使用 Ubuntu 24.04 / Python 3.12、`python3-venv`
和[工作台 CI](../../.github/workflows/capture-console.yml)列出的 Qt 系统库运行：

```bash
python3 tools/release/verify_distributions.py \
  --profile all --output .runtime/release/distribution-check-1
```

输出目录必须不存在；再次运行时换一个目录名。过程需要访问公开 Python 包索引，
会在临时目录创建构建环境和安装环境，结束后移除环境，保留日志、安装包、截图与
`report.json`。不会调用桌面安装器或启动传感器。

完整检查先为 12 个 Python 包生成 sdist，再从 sdist 构建 wheel，检查许可证、已有
第三方声明及包内 JSON/QSS/SVG/XML 资源，随后在源码目录之外进行非 editable 安装，
运行依赖检查、导入和命令行检查、中英文工作台演示以及合成手套演示。
PySide6 与 PyQt5 使用不同虚拟环境，避免本机已有安装掩盖依赖或资源缺失。

可用 `--profile core`、`offline` 或 `console` 缩小检查范围。
`--build-only` 只检查构建产物，不能作为安装和运行验证。
`constraints-tested.txt` 固定部分参考依赖；报告记录实际解析的包名、版本及产物
SHA-256，但这不是完整依赖锁或逐字节可重复构建承诺。

这项检查不构建 CMake ROS 包，也不运行 ROS 节点、真实 Tracker、外部 FSGlove
数值核心或 MANO 模型。ROS 的普通安装与测试仍须按
[贡献指南](../../CONTRIBUTING.md)单独执行；离屏截图不代表真实显卡或硬件验收。
安装包是本地验证产物，分发前仍须检查对应第三方许可要求。

## 维护测试

```bash
python3 -m unittest discover -s tools/release/tests -v
python3 tools/test_check_public_tree.py
```

维护测试使用 Python 3.12；合成归档和临时仓库测试不连接设备，也不读取真实
Capture/Bundle。运行时软件测试与各硬件验收见
[CONTRIBUTING](../../CONTRIBUTING.md) 和[最终用户手册](../../docs/user-manual.md)。
