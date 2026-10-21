"""Small bilingual installation view, independent of the workstation Qt runtime."""
from __future__ import annotations

import locale
import os
from pathlib import Path
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import ttk


COPY = {
    "title": ("安装 Vision Tactile", "Install Vision Tactile"),
    "subtitle": ("采集工作台与 Tracker", "Capture workbench and Tracker"),
    "intro": ("自动准备运行环境、相机驱动和 Tracker 依赖。\n首次安装需要网络，系统会请求管理员密码。",
              "Set up the runtime, camera drivers and Tracker dependencies.\nThe first installation needs Internet access and administrator approval."),
    "ready": ("准备安装", "Ready to install"),
    "start": ("安装", "Install"),
    "retry": ("重试", "Retry"),
    "cancel": ("停止", "Stop"),
    "close": ("关闭", "Close"),
    "open": ("打开工作台", "Open workbench"),
    "details": ("查看详细进度", "Show details"),
    "working": ("正在准备，请稍候…", "Preparing, please wait…"),
    "stopping": ("正在停止；系统软件安装会在当前步骤完成后停止。", "Stopping after the current system installation step finishes."),
    "complete": ("安装完成", "Installation complete"),
    "done": ("可以打开工作台，连接设备并按初始化向导操作。\n以后从应用菜单启动，无需保留下载目录。",
             "Open the workbench, connect your devices and follow the initialization wizard.\nUse the application menu next time. The downloaded folder is no longer needed."),
    "failed": ("安装未完成", "Installation did not complete"),
    "error": ("请查看下方原因；解决后可重试。", "Review the details below, resolve the issue and retry."),
    "note": ("Ubuntu 24.04 · x86_64　安装不会启动传感器。", "Ubuntu 24.04 · x86_64   Installation does not start sensors."),
}

PHASES = {
    "preflight": ("正在检查安装条件", "Checking installation requirements"),
    "stage": ("正在准备程序文件", "Preparing application files"),
    "system": ("正在安装系统依赖", "Installing system dependencies"),
    "ros": ("正在安装运行环境", "Installing the runtime"),
    "sdk": ("正在准备相机运行库", "Preparing the camera runtime"),
    "driver": ("正在准备相机驱动", "Preparing the camera driver"),
    "workspace": ("正在构建采集组件", "Building capture components"),
    "permissions": ("正在配置设备访问", "Configuring device access"),
    "verify": ("正在检查运行环境", "Checking the runtime"),
    "console": ("正在安装工作台", "Installing the workbench"),
    "tracker": ("正在安装 Tracker 依赖", "Installing Tracker dependencies"),
    "desktop": ("正在创建启动入口", "Creating the application launcher"),
    "complete": COPY["complete"],
}


def launch_workbench(result):
    source = Path(result["source_root"])
    env = dict(os.environ, VT_CONSOLE_PYTHON=str(result["python"]))
    original = env.pop("LD_LIBRARY_PATH_ORIG", None)
    env.pop("LD_LIBRARY_PATH", None)
    if original:
        env["LD_LIBRARY_PATH"] = original
    for name in ("PYTHONHOME", "PYTHONPATH", "_MEIPASS2"):
        env.pop(name, None)
    return subprocess.Popen(["/bin/bash", str(source / "tools/capture_deploy/start_workbench.sh")],
                            env=env, start_new_session=True, stdin=subprocess.DEVNULL)


class InstallerWindow:
    def __init__(self, root, directory, *, installer_factory=None):
        self.root, self.directory = root, Path(directory)
        self.factory = installer_factory
        self.english = not (locale.getlocale()[0] or "").lower().startswith("zh")
        self.events = queue.Queue()
        self.cancel = threading.Event()
        self.running = False
        self.result = None
        self.state = "ready"
        self.status_key = "ready"
        self.phase = None
        self.thread = None
        self.close_when_stopped = False
        root.configure(bg="#000000")
        root.geometry("660x400")
        root.minsize(600, 380)
        root.protocol("WM_DELETE_WINDOW", self._close)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TProgressbar", troughcolor="#303030", background="#ffffff",
                        bordercolor="#000000", lightcolor="#ffffff", darkcolor="#ffffff")
        self.frame = tk.Frame(root, bg="#000000", padx=30, pady=24)
        self.frame.pack(fill="both", expand=True)
        header = tk.Frame(self.frame, bg="#000000")
        header.pack(fill="x")
        self.title = tk.Label(header, bg="#000000", fg="#ffffff", anchor="w", font=("sans", 21, "bold"))
        self.title.pack(side="left")
        self.language = tk.Button(header, command=self._language, bg="#141414", fg="#dddddd",
                                  activebackground="#303030", activeforeground="#ffffff", relief="flat")
        self.language.pack(side="right")
        self.subtitle = tk.Label(self.frame, bg="#000000", fg="#aaaaaa", anchor="w", font=("sans", 11))
        self.subtitle.pack(fill="x", pady=(5, 20))
        self.intro = tk.Label(self.frame, bg="#000000", fg="#cccccc", anchor="w", justify="left",
                              wraplength=590, font=("sans", 11))
        self.intro.pack(fill="x")
        self.status = tk.Label(self.frame, bg="#000000", fg="#ffffff", anchor="w", justify="left",
                               wraplength=590, font=("sans", 12, "bold"))
        self.status.pack(fill="x", pady=(26, 10))
        self.progress = ttk.Progressbar(self.frame, mode="indeterminate")
        self.progress.pack(fill="x", ipady=0)
        self.details_button = tk.Button(self.frame, command=self._details, bg="#000000", fg="#aaaaaa",
                                        activebackground="#141414", activeforeground="#ffffff", relief="flat")
        self.details_button.pack(anchor="w", pady=(8, 0))
        self.log = tk.Text(self.frame, height=10, bg="#080808", fg="#cccccc", relief="flat",
                           wrap="word", state="disabled", font=("monospace", 9))
        self.note = tk.Label(self.frame, bg="#000000", fg="#888888", anchor="w", wraplength=590, font=("sans", 9))
        self.note.pack(fill="x", pady=(14, 8))
        buttons = tk.Frame(self.frame, bg="#000000")
        buttons.pack(fill="x", side="bottom")
        self.secondary = tk.Button(buttons, command=self._close, bg="#141414", fg="#dddddd", relief="flat", padx=18, pady=8)
        self.secondary.pack(side="right")
        self.primary = tk.Button(buttons, command=self._primary, bg="#ffffff", fg="#000000", relief="flat", padx=22, pady=8)
        self.primary.pack(side="right", padx=(0, 10))
        self._translate()
        root.after(100, self._poll)

    def text(self, key):
        return COPY[key][int(self.english)]

    def _translate(self):
        self.root.title(self.text("title"))
        for widget, key in ((self.title, "title"), (self.subtitle, "subtitle"), (self.note, "note"),
                            (self.details_button, "details")):
            widget.configure(text=self.text(key))
        self.intro.configure(text=self.text("done" if self.result else "intro"))
        self.language.configure(text="中文" if self.english else "English")
        self.primary.configure(text=self.text("open" if self.result else "retry" if self.state == "failed" else "start"))
        self.secondary.configure(text=self.text("cancel" if self.running else "close"))
        status = PHASES[self.phase][int(self.english)] if self.running and self.phase in PHASES else self.text(self.status_key)
        if self.cancel.is_set() and self.running:
            status = self.text("stopping")
        self.status.configure(text=status)

    def _language(self):
        self.english = not self.english
        self._translate()

    def _primary(self):
        if self.result:
            try:
                launch_workbench(self.result)
            except OSError as exc:
                self._append(str(exc))
                self._details(show=True)
                return
            self.root.destroy()
            return
        if self.running:
            return
        self.running = True
        self.state = "running"
        self.status_key = "working"
        self.phase = None
        self.cancel.clear()
        self.primary.configure(state="disabled")
        self.progress.start(12)
        self._translate()
        self.thread = threading.Thread(target=install_worker,
                                       args=(self.directory, self.events, self.cancel.is_set, self.factory),
                                       name="workstation-installer", daemon=False)
        self.thread.start()

    def _append(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text.rstrip() + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 300:
            self.log.delete("1.0", "100.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _poll(self):
        for _ in range(200):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            kind = event.get("kind")
            if kind in {"progress", "log"}:
                message = event.get("message") or event.get("text", "")
                if kind == "progress":
                    self.phase = event.get("phase")
                    self._translate()
                if message:
                    self._append(message)
            elif kind in {"complete", "failed"}:
                self.running = False
                self.progress.stop()
                self.primary.configure(state="normal")
                if kind == "complete":
                    self.result = event["result"]
                    self.state = self.status_key = "complete"
                    self.progress.configure(mode="determinate", value=100)
                else:
                    self.state = self.status_key = "failed"
                    self._append(event.get("message", self.text("error")))
                    self._details(show=True)
                self._translate()
                if self.close_when_stopped:
                    self.root.destroy()
                    return
        self.root.after(100, self._poll)

    def _details(self, show=False):
        if show or not self.log.winfo_manager():
            self.log.pack(fill="both", expand=True, before=self.note, pady=8)
            self.root.geometry("660x600")
        else:
            self.log.pack_forget()
            self.root.geometry("660x400")

    def _close(self):
        if self.running:
            self.cancel.set()
            self.status_key = "stopping"
            self.status.configure(text=self.text("stopping"))
            self.close_when_stopped = True
        else:
            self.root.destroy()


def install_worker(directory, events, cancelled, factory=None):
    """Own installation state only; no Tk objects cross the thread boundary."""
    try:
        from .engine import Installer
        result = (factory or Installer)(directory, notify=events.put, cancel=cancelled).run()
        events.put({"kind": "complete", "result": result})
    except Exception as exc:
        events.put({"kind": "failed", "message": str(exc)})


def run(directory, *, smoke_test=False):
    root = tk.Tk()
    InstallerWindow(root, directory)
    if smoke_test:
        root.after(200, root.destroy)
    root.mainloop()
    return 0
