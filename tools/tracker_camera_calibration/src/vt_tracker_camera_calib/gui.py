"""Thin offline Tk entry point over the existing calibration workflow.

This first desktop entry intentionally does not connect LiveMonitor or expose
record_command. It never subscribes, records, or opens hardware on startup.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import stat

from vt_capture_core.calibration_policy import board_geometry_defaults

from . import gui_workflow as workflow
from .charuco import create_board, render_board
from .config import load_config
from .config_writer import REFERENCE_CAMERAS, TRACKER_ROLES


MAX_REPORT_BYTES = 2 * 1024 * 1024
MAX_LOG_BYTES = 128 * 1024


def _read_text(path, limit, *, tail=False):
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or (not tail and info.st_size > limit):
            raise ValueError("报告或日志必须是大小受限的普通文件")
        if tail:
            stream.seek(max(0, info.st_size - limit))
        return stream.read(limit).decode("utf-8", errors="replace")


class OfflineSession:
    """GUI-independent state; only explicit solve() launches an owned CLI job."""
    def __init__(self, output_root, *, job_factory=workflow.ManagedJob):
        self.output_root = Path(output_root).expanduser()
        self.config_path = self.config = None
        self.job = job_factory()
        self.state = "IDLE"
        self.detail = "离线模式：配置、板图预览、已有 bag 求解与报告。在线预览／专用录包尚未接入。"
        self.output_path = None
        self.report = None
        self.log = ""
        self._log_path = None
        self._active = self._cancelled = False

    @property
    def running(self):
        return self._active

    def _idle(self):
        if self.running:
            raise ValueError("请先等待本次求解结束；不会并发运行或切换配置")

    def select_config(self, path):
        self._idle()
        path = Path(path).expanduser().resolve(strict=True)
        config = load_config(path)
        create_board(config.board)
        self.config_path, self.config = path, config
        self.state, self.report, self.log, self._log_path = "IDLE", None, "", None
        self.detail = f"已读取配置：{config.camera_name} / {config.tracker_role}；尚未求解。"
        return path

    def prepare(self, camera, role, square_mm, marker_mm, *, squares_x=None, squares_y=None, dictionary=None):
        self._idle()
        path = workflow.prepare_config(self.output_root, camera, role, square_mm, marker_mm,
                                       squares_x=squares_x, squares_y=squares_y, dictionary=dictionary)
        return self.select_config(path)

    def import_configuration(self, source):
        self._idle()
        return self.select_config(workflow.import_config(self.output_root, Path(source)))

    def board_image(self):
        if self.config is None:
            raise ValueError("请先读取或生成配置")
        from PIL import Image
        board = create_board(self.config.board)
        # Preview has a fixed pixel bound, independent of physical board size.
        size = (640, 420)
        value = (board.generateImage(size, marginSize=12, borderBits=1) if hasattr(board, "generateImage")
                 else board.draw(size, marginSize=12, borderBits=1))
        return Image.fromarray(value)

    def export_board(self, output):
        self._idle()
        if self.config is None:
            raise ValueError("请先读取或生成配置")
        board = self.config.board
        pixels = board.squares_x * board.squares_y * board.square_length_m**2 * (300 / .0254)**2
        if pixels > 20_000_000:
            raise ValueError("打印板超过本界面 2000 万像素上限，请核对实测尺寸")
        return render_board(board, workflow._new_output(Path(output)), 300)

    def solve(self, bag, output, *, closed_confirmed=False):
        self._idle()
        if not closed_confirmed:
            raise ValueError("必须明确确认所选 bag 已结束录制并关闭")
        if self.config_path is None:
            raise ValueError("请先读取或生成配置")
        bag, output = Path(bag).expanduser().resolve(), Path(output).expanduser().absolute()
        if not (bag / "metadata.yaml").is_file():
            raise ValueError("请选择包含 metadata.yaml 的已结束 rosbag2 目录")
        if output.resolve().is_relative_to(bag):
            raise ValueError("结果不能写入源 bag 内")
        # Validate destinations before allocating an immutable per-run config.
        workflow.solver_command(self.config_path, bag, output)
        snapshot = workflow.import_config(self.output_root, self.config_path)
        argv = workflow.solver_command(snapshot, bag, output)
        self.job.start(argv, snapshot.parent / "solver.log")
        self._log_path = self.job.log_path
        self.output_path = output
        self.state, self.detail = "RUNNING", "正在运行现有离线求解器；不会启动传感器或 Recorder。"
        self._active, self._cancelled = True, False
        self.report, self.log = None, ""

    def cancel(self):
        if self._active and self.job.running:
            self.job.stop()
            self._cancelled = True
            self.state, self.detail = "CANCELLING", "已请求中断本次离线计算，等待子进程退出；不影响采集设备。"

    def load_report(self, path):
        report = json.loads(_read_text(Path(path), MAX_REPORT_BYTES))
        if (not isinstance(report, dict) or report.get("quality") not in {"TARGET", "ACCEPTABLE", "REJECTED"}
                or not isinstance(report.get("counts"), dict) or not isinstance(report.get("metrics"), dict)):
            raise ValueError("这不是本工具的标定质量报告")
        self.report = report
        return report

    def poll(self):
        if self._log_path:
            try:
                self.log = _read_text(self._log_path, MAX_LOG_BYTES, tail=True)
            except (OSError, ValueError) as exc:
                self.log = f"日志暂不可读：{exc}"
        if not self._active:
            return
        code = self.job.poll()
        if code is None:
            return
        self._active = False
        if self._cancelled:
            self.state, self.detail = "CANCELLED", "计算已取消；可能留下诊断文件，重试请选择新的结果目录。"
            return
        try:
            report = self.load_report(self.output_path / "report.json")
        except (OSError, ValueError) as exc:
            self.state, self.detail = "FAILED", f"求解退出 {code}，未取得有效质量报告：{exc}"
            return
        if code == 0 and report["quality"] in {"TARGET", "ACCEPTABLE"}:
            self.state = "COMPLETE"
            self.detail = f"本次求解：{report['quality']}。仍需独立重复性验证；不会自动应用外参。"
        else:
            self.state, self.detail = "FAILED", f"求解未通过（退出 {code}，质量 {report['quality']}）；仅保留诊断，不可作为可用外参。"


class CalibrationWindow:
    def __init__(self, root, *, output_root=None, config_path=None, session=None):
        import tkinter as tk
        from tkinter import ttk
        self.root, self.tk, self.ttk = root, tk, ttk
        self.session = session or OfflineSession(output_root or Path.home() / "visiontactile-data/calibration/gui")
        self._closing = False
        self._after = None
        self._last_log = self._last_report = None
        self._photo = None
        root.title("Vision Tactile · 离线相机–Tracker 标定")
        width = min(1120, max(850, root.winfo_screenwidth() - 60))
        height = min(740, max(620, root.winfo_screenheight() - 100))
        root.geometry(f"{width}x{height}")
        root.minsize(850, 620)
        root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TFrame", background="#f4f5f7")
        style.configure("TLabel", background="#f4f5f7", font=("Noto Sans CJK SC", 10))
        style.configure("TButton", padding=(10, 6))
        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="离线标定工作台", font=("Noto Sans CJK SC", 18, "bold")).pack(anchor="w")
        ttk.Label(outer, text="只处理已结束的 bag；在线监视与专用录包尚未接入。本窗口不会连接硬件、订阅 ROS 或自动应用外参。",
                  wraplength=1050).pack(anchor="w", pady=(6, 12))
        panes = ttk.Panedwindow(outer, orient="horizontal")
        panes.pack(fill="both", expand=True)
        left, right = ttk.Frame(panes, padding=8), ttk.Frame(panes, padding=8)
        panes.add(left, weight=1)
        panes.add(right, weight=2)
        geometry = board_geometry_defaults()
        self.variables = {key: tk.StringVar(value=value) for key, value in dict(
            output_root=str(self.session.output_root), config="", camera="", role="",
            square=f'{geometry["square_length_mm"]:g}', marker=f'{geometry["marker_length_mm"]:g}',
            squares_x=str(geometry["squares_x"]), squares_y=str(geometry["squares_y"]),
            dictionary=geometry["dictionary"], bag="", output="").items()}
        self._editable = []
        tabs = ttk.Notebook(left)
        tabs.pack(fill="both", expand=True)
        config_shell, solve_page = ttk.Frame(tabs), ttk.Frame(tabs, padding=10)
        canvas = tk.Canvas(config_shell, highlightthickness=0, background="#f4f5f7", width=360)
        scroll = ttk.Scrollbar(config_shell, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        config_page = ttk.Frame(canvas, padding=10)
        canvas_item = canvas.create_window((0, 0), window=config_page, anchor="nw")
        config_page.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(canvas_item, width=event.width))
        tabs.add(config_shell, text="配置与板图")
        tabs.add(solve_page, text="已有 bag 求解")
        self._path_row(config_page, "会话数据根目录", "output_root", "directory")
        self._path_row(config_page, "已有配置", "config", "file")
        self._button(config_page, "导入配置并保存独立快照", self.import_configuration)
        ttk.Separator(config_page).pack(fill="x", pady=10)
        self._combo(config_page, "相机（手动确认物理刚体）", "camera", tuple(REFERENCE_CAMERAS))
        self._combo(config_page, "同支架 Tracker 角色", "role", TRACKER_ROLES)
        for title, key in (("实测方格边长 mm", "square"), ("实测 Marker 边长 mm", "marker"),
                           ("横向方格数", "squares_x"), ("纵向方格数", "squares_y"), ("ArUco 字典", "dictionary")):
            self._entry(config_page, title, key)
        ttk.Label(config_page, text="默认参数对应参考标定板，请核对实测尺寸；相机与角色不自动配对。", wraplength=330).pack(anchor="w", pady=6)
        self.measured = tk.BooleanVar(value=False)
        check = ttk.Checkbutton(config_page, text="已核对刚体身份与实测板材尺寸", variable=self.measured)
        check.pack(anchor="w")
        self._editable.append(check)
        self._button(config_page, "生成配置与板图预览", self.prepare_configuration)
        self._button(config_page, "导出 300 DPI 板图 PNG（不覆盖）", self.export_board)
        self._path_row(solve_page, "已关闭的 bag 目录", "bag", "directory")
        self._path_row(solve_page, "新的结果目录", "output", "new_directory")
        self.closed_confirmed = tk.BooleanVar(value=False)
        self.variables["bag"].trace_add("write", lambda *_: self.closed_confirmed.set(False))
        check = ttk.Checkbutton(solve_page, text="确认所选 bag 已结束录制并完整关闭", variable=self.closed_confirmed)
        check.pack(anchor="w", pady=8)
        self._editable.append(check)
        ttk.Label(solve_page, text="使用当前已导入／生成配置。复用现有手眼算法和质量门禁；至少 40 个合格静止姿态。", wraplength=330).pack(anchor="w", pady=8)
        self._button(solve_page, "开始离线求解", self.start_solve)
        self.cancel_button = ttk.Button(solve_page, text="取消本次计算", command=self.session.cancel)
        self.cancel_button.pack(fill="x", pady=4)
        self._button(solve_page, "查看已有 report.json", self.choose_report)
        self.identity = tk.StringVar(value="尚未选取配置")
        ttk.Label(right, textvariable=self.identity, wraplength=600).pack(anchor="w", pady=(0, 8))
        self.board = ttk.Label(right, text="配置后显示 ChArUco 板图预览（不是尺寸校验）", anchor="center")
        self.board.pack(fill="x", pady=5)
        report_tabs = ttk.Notebook(right)
        report_tabs.pack(fill="both", expand=True, pady=8)
        self.report_text = self._text_tab(report_tabs, "质量报告")
        self.log_text = self._text_tab(report_tabs, "求解日志（末尾 128 KiB）")
        self.status = tk.StringVar(value=self.session.detail)
        ttk.Label(outer, textvariable=self.status, wraplength=1050).pack(fill="x", pady=(10, 0))
        if config_path is not None:
            try:
                self.session.select_config(config_path)  # Read-only; no auto session creation.
                self._show_config()
            except Exception as exc:
                self.status.set(f"配置未加载：{exc}")
        self._refresh()

    def _entry(self, parent, title, key):
        self.ttk.Label(parent, text=title).pack(anchor="w")
        widget = self.ttk.Entry(parent, textvariable=self.variables[key])
        widget.pack(fill="x", pady=(0, 5))
        self._editable.append(widget)

    def _combo(self, parent, title, key, choices):
        self.ttk.Label(parent, text=title).pack(anchor="w")
        widget = self.ttk.Combobox(parent, textvariable=self.variables[key], values=choices, state="readonly")
        widget.pack(fill="x", pady=(0, 6))
        self._editable.append(widget)

    def _button(self, parent, title, action):
        button = self.ttk.Button(parent, text=title, command=lambda: self._action(action))
        button.pack(fill="x", pady=4)
        self._editable.append(button)
        return button

    def _path_row(self, parent, title, key, kind):
        self._entry(parent, title, key)
        def browse():
            from tkinter import filedialog
            initial = self.variables[key].get() or str(Path.home())
            if kind == "file":
                value = filedialog.askopenfilename(parent=self.root, initialdir=str(Path(initial).parent), filetypes=[("YAML", "*.yaml *.yml"), ("所有文件", "*")])
            elif kind == "new_directory":
                value = filedialog.asksaveasfilename(parent=self.root, title="指定新的结果目录名称", initialdir=str(Path(initial).parent))
            else:
                value = filedialog.askdirectory(parent=self.root, initialdir=initial)
            if value:
                self.variables[key].set(value)
        self._button(parent, "选择…", browse)

    def _text_tab(self, notebook, title):
        frame = self.ttk.Frame(notebook)
        notebook.add(frame, text=title)
        text = self.tk.Text(frame, wrap="word", height=10, state="disabled", font=("Noto Sans Mono", 10))
        scrollbar = self.ttk.Scrollbar(frame, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        text.pack(fill="both", expand=True)
        return text

    @staticmethod
    def _set_text(widget, value):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", value)
        widget.configure(state="disabled")

    def _action(self, action):
        try:
            action()
            self.status.set(self.session.detail)
        except Exception as exc:
            self.status.set(f"未完成：{exc}")

    def _show_config(self):
        from PIL import ImageTk
        config = self.session.config
        self.variables["config"].set(str(self.session.config_path))
        self.identity.set(f"{config.camera_name} · {config.camera_serial}  ↔  {config.tracker_role}\n"
                          f"{config.board.squares_x}×{config.board.squares_y} · "
                          f"{config.board.square_length_m * 1000:g} / {config.board.marker_length_m * 1000:g} mm\n"
                          "板图仅供检查，打印请使用导出的带 DPI PNG。")
        self._photo = ImageTk.PhotoImage(self.session.board_image().resize((480, 315)), master=self.root)
        self.board.configure(image=self._photo, text="")

    def prepare_configuration(self):
        if not self.measured.get():
            raise ValueError("请先确认物理刚体身份和实测尺寸，不使用默认数值猜测")
        values = {key: variable.get().strip() for key, variable in self.variables.items()}
        self.session.output_root = Path(values["output_root"]).expanduser()
        self.session.prepare(values["camera"], values["role"], float(values["square"]), float(values["marker"]),
                             squares_x=int(values["squares_x"]), squares_y=int(values["squares_y"]), dictionary=values["dictionary"])
        self._show_config()

    def import_configuration(self):
        self.session.output_root = Path(self.variables["output_root"].get()).expanduser()
        self.session.import_configuration(self.variables["config"].get())
        self._show_config()

    def export_board(self):
        from tkinter import filedialog
        value = filedialog.asksaveasfilename(parent=self.root, title="导出新的板图 PNG", defaultextension=".png", filetypes=[("PNG", "*.png")])
        if value:
            self.session.export_board(value)
            self.session.detail = "板图已导出；打印必须保持原尺寸，打印后重新测量。"

    def start_solve(self):
        self.session.output_root = Path(self.variables["output_root"].get()).expanduser()
        # A changed text path is never silently ignored in favour of an older config.
        self.session.select_config(self.variables["config"].get())
        self.session.solve(self.variables["bag"].get(), self.variables["output"].get(), closed_confirmed=self.closed_confirmed.get())

    def choose_report(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(parent=self.root, title="查看既有标定报告", filetypes=[("标定质量报告", "*.json")])
        if path:
            self.session.load_report(path)
            self.session.detail = "正在查看既有报告；这不代表新求解或重复性验证通过。"

    def _refresh(self):
        self.session.poll()
        running = self.session.running
        for widget in self._editable:
            widget.state(["disabled"] if running else ["!disabled"])
        self.cancel_button.state(["!disabled"] if running else ["disabled"])
        if self.session.state != "IDLE":
            self.status.set(self.session.detail)
        if self.session.log != self._last_log:
            self._set_text(self.log_text, self.session.log)
            self._last_log = self.session.log
        report = json.dumps(self.session.report, ensure_ascii=False, indent=2) if self.session.report else "尚无标定质量报告。"
        if report != self._last_report:
            self._set_text(self.report_text, report)
            self._last_report = report
        if self._closing and not running:
            self.root.destroy()
            return
        self._after = self.root.after(150, self._refresh)

    def close(self):
        self._closing = True
        if self.session.running:
            self.session.cancel()
            self.status.set("正在等待本窗口的离线计算退出；未停止任何设备。")
        else:
            if self._after:
                self.root.after_cancel(self._after)
            self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description="离线相机–Tracker 标定 GUI；不连接硬件、不录包")
    parser.add_argument("--output-root", type=Path, default=Path.home() / "visiontactile-data/calibration/gui")
    parser.add_argument("--config", type=Path, help="只读加载已有配置；不会自动求解")
    args = parser.parse_args(argv)
    import tkinter as tk
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        parser.exit(1, f"无法打开图形界面：{exc}\n需要本机桌面显示与 python3-tk；未启动硬件。\n")
    CalibrationWindow(root, output_root=args.output_root, config_path=args.config)
    root.mainloop()
    return 0


def entrypoint():
    raise SystemExit(main())


if __name__ == "__main__":
    entrypoint()
