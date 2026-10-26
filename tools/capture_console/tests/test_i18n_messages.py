import json
from pathlib import Path
from string import Formatter

import pytest

from vt_capture_console import i18n
from vt_capture_console.i18n import messages


@pytest.fixture(autouse=True)
def diagnostic_language():
    before = i18n.language()
    messages._index.cache_clear()
    messages._alias_index.cache_clear()
    messages._resolve.cache_clear()
    i18n.set_language("en", persist=False)
    # Existing Qt widgets may render diagnostics during the language change.
    messages._index.cache_clear()
    messages._alias_index.cache_clear()
    messages._resolve.cache_clear()
    yield
    i18n.set_language(before, persist=False)
    messages._index.cache_clear()
    messages._alias_index.cache_clear()
    messages._resolve.cache_clear()


def test_known_diagnostic_retranslates_and_preserves_original_payload():
    snapshot = {"message": "后台响应超时", "id": "用户自定义/相机_2"}
    before = snapshot.copy()
    displayed = messages.message(snapshot["message"])
    assert isinstance(displayed, i18n.LocalizedText)
    assert str(displayed) == "Backend response timeout"
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == "后台响应超时"
    i18n.set_language("en", persist=False)
    assert str(displayed) == "Backend response timeout"
    assert snapshot == before


@pytest.mark.parametrize("source,english", [
    ("历史 Tracker 位姿", "Historical Tracker poses"),
    ("该时刻无空间数据；已有数据可在“话题详情”查看。",
     "No spatial data at this time; available data can be viewed in “Topic details”."),
])
def test_concise_archive_spatial_status_is_translated(source, english):
    displayed = messages.message(source)
    assert str(displayed) == english
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == source


@pytest.mark.parametrize("source,english", [
    ("此手已有外部发布程序；窗口关闭后后台仍可能运行。请退出该外部程序后重试。",
     "An external program is publishing this hand's data; its background process may still run after the window closes. Exit that program and try again."),
    ("串口路径未配置；请检查设备启动配置。",
     "No serial port is configured; check the device launch configuration."),
    ("串口 /dev/serial/用户-{port} 已断开或不存在；请检查 USB 连接和配置端口。",
     "Serial port /dev/serial/用户-{port} is disconnected or missing; check the USB connection and configured port."),
    ("无法检查串口 /dev/serial/用户-{port}：test OS error；设备未启动。",
     "Cannot inspect serial port /dev/serial/用户-{port}: test OS error; the device was not started."),
    ("无法检查串口 /dev/serial/用户-{port} 的有效身份读写权限；设备未启动。",
     "Cannot check serial port /dev/serial/用户-{port} read/write access for the current process; the device was not started."),
])
def test_glove_launch_blockers_translate_without_changing_device_paths(source, english):
    displayed = messages.message(source)
    assert str(displayed) == english
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == source


def test_complete_template_keeps_path_braces_quotes_and_chinese_opaque():
    path = "/home/操作者/录制/{自定义}/'试次一'.json"
    raw = "拒绝读取非私有或过大的设置文件：" + path
    displayed = messages.message(raw)
    assert str(displayed) == (
        "Refusing to read settings file with non-private permissions or excessive size: " + path)
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == raw


def test_complete_authored_concatenation_keeps_user_batch_name_opaque():
    name = "后台响应超时 · 我的录制/相机 {v0}"
    raw = "已创建演示批次：" + name
    assert str(messages.message(raw)) == "Demo batch created: " + name


@pytest.mark.parametrize("raw", [
    "UNRECOGNIZED / 未知错误 42",
    "我的实验笔记：后台响应超时；保留这段原文。",
    "/home/相机/后台响应超时.txt",
    "{v0!r} 未知模板",
    "",
])
def test_unknown_text_is_not_partially_translated(raw):
    assert str(messages.message(raw)) == raw


def test_numeric_repr_and_reordered_placeholders_are_not_formatted_twice(monkeypatch):
    source = "诊断样本：{v0!r}，延迟 {v1:.1f} ms。"
    english = "Delay {v1:.1f} ms; sample {v0!r}."
    monkeypatch.setattr(messages, "_load_sources", lambda: (source,))
    monkeypatch.setattr(messages, "catalog", lambda: {source: english})
    raw = source.format(v0="用户路径/录制", v1=12.34)
    displayed = messages.message(raw)
    assert str(displayed) == "Delay 12.3 ms; sample '用户路径/录制'."
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == raw


def test_ambiguous_capture_boundary_passes_through(monkeypatch):
    source = "样本 {v0} / {v1} 无效"
    monkeypatch.setattr(messages, "_load_sources", lambda: (source,))
    monkeypatch.setattr(messages, "catalog", lambda: {source: "Invalid {v1}: {v0}"})
    raw = "样本 user / nested / path 无效"
    assert str(messages.message(raw)) == raw


def test_kernel_failure_template_retains_exact_controller_addresses():
    addresses = "0000:80:14.0, 0000:81:00.0"
    raw = (f"内核报告 USB 控制器 {addresses} 失效（HC died），当前未枚举到配置的相机。"
           "已阻止相机启动，避免驱动持续重试；控制器恢复且相机重新枚举后可再次启动。")
    translated = str(messages.message(raw))
    assert translated.startswith(f"The kernel reported USB controller {addresses} failure (HC died)")
    assert "Camera startup is blocked" in translated
    assert addresses in translated
    i18n.set_language("zh_CN", persist=False)
    assert str(messages.message(raw)) == raw


def test_allowlist_has_translations_and_stays_bounded():
    exact, templates = messages._index()
    assert exact <= i18n.catalog().keys()
    assert len(exact) <= messages.MAX_SOURCES
    assert len(templates) <= messages.MAX_TEMPLATES
    assert templates


@pytest.mark.parametrize("prefix,english", [
    ("", ""),
    ("正在安全停止录制：", "Stopping recording safely: "),
    ("录制停止尚未确认：", "Recorder termination is not yet confirmed: "),
    ("录制已提前结束并完成收尾：", "Recording ended early and finalization completed: "),
])
def test_recording_disk_threshold_reason_is_fully_translated(prefix, english):
    raw = prefix + "磁盘空间达到停止阈值：可用 1.50 GiB，预留 2.28 GiB。"
    translated = messages.message(raw)
    assert str(translated) == (english + "Disk space reached the stop threshold: "
                              "1.50 GiB available; 2.28 GiB reserved.")
    i18n.set_language("zh_CN", persist=False)
    assert str(translated) == raw


def test_recording_capacity_estimate_keeps_numbers_and_unknown_topic_limit():
    raw = ("磁盘空间不足：可用 33.00 GiB，预计至少需要 129.74 GiB（含停止预留 2.28 GiB）。 "
           "磁盘容量预算仅包含已知图像流；追加话题未配置容量估计。")
    assert str(messages.message(raw)) == (
        "Insufficient disk space: 33.00 GiB available; at least 129.74 GiB required, "
        "including 2.28 GiB reserved for stopping. "
        "The capacity budget covers known image streams only; additional topics have no configured size estimate.")


def test_recording_disk_error_preserves_raw_error_and_path():
    error = "Permission denied: '/数据/原始记录/{试次}'"
    raw = "录制停止尚未确认：磁盘空间查询失败：" + error
    translated = messages.message(raw)
    assert str(translated) == "Recorder termination is not yet confirmed: Disk space check failed: " + error
    i18n.set_language("zh_CN", persist=False)
    assert str(translated) == raw


@pytest.mark.parametrize("prefix,english", [
    ("无法检查本次必需数据流：", "Cannot check required streams for this recording: "),
    ("本次必需数据流尚未就绪：", "Required streams are not ready for this recording: "),
    ("开始前已接收的数据流已中断：", "Streams receiving data before start have stopped: "),
])
def test_plan_diagnostics_preserve_opaque_topic_evidence(prefix, english):
    evidence = "/vive/left_wrist/sample, /用户/'原始{v0}'/后台响应超时"
    payload = {"code": "capture_streams_not_ready", "detail": prefix + evidence,
               "topic_evidence": evidence}
    original = payload.copy()
    displayed = messages.message(payload["detail"])
    assert str(displayed) == english + evidence
    assert payload == original
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == original["detail"]
    assert payload == original


def test_all_plan_diagnostics_are_registered_and_retranslate():
    path = Path(messages.__file__).with_name("catalog_plan_diagnostics.json")
    entries = json.loads(path.read_text(encoding="utf-8"))
    exact, _ = messages._index()
    assert entries.keys() <= exact
    for source, english in entries.items():
        raw = source.format(topics="/d405_1/color/image_raw")
        translated = messages.message(raw)
        assert str(translated) == english.format(topics="/d405_1/color/image_raw")
        i18n.set_language("zh_CN", persist=False)
        assert str(translated) == raw
        i18n.set_language("en", persist=False)


@pytest.mark.parametrize("raw,english", [
    ("当前没有可录制的数据流，请先启动至少一个采集设备。",
     "No data streams are available to record. Start at least one capture device."),
    ("已保存的采集计划无效，未发送录制命令。",
     "The saved capture plan is invalid. No recording command was sent."),
])
def test_capture_admission_rejection_has_an_actionable_translation(raw, english):
    assert str(messages.message(raw)) == english


def test_oversized_input_is_unchanged_and_not_cached():
    raw = "配置未保存：" + "用户数据" * messages.MAX_MESSAGE_LENGTH
    assert str(messages.message(raw)) == raw
    assert messages._resolve.cache_info().currsize == 0


def test_resolution_cache_has_a_fixed_capacity():
    for index in range(messages.MAX_CACHED_MESSAGES + 20):
        messages.message(f"未知错误 {index}").render()
    assert messages._resolve.cache_info().currsize == messages.MAX_CACHED_MESSAGES


def test_already_localized_display_keeps_its_original_recipe():
    display = i18n.tr("后台响应超时")
    assert messages.message(display) is display


@pytest.mark.parametrize("raw,chinese", [
    ("An earlier calibration is still active.", "上一项标定仍在进行，请等待完成或取消。"),
    ("Tracker has no current valid identity; start the selected device before calibration",
     "尚未取得追踪器的有效身份，请先启动所选设备再进行标定。"),
    ("camera_identity_unverified", "相机身份尚未核验，请检查相机连接。"),
    ("Live calibration identity not verified: camera_identity_unverified",
     "实时标定设备身份尚未核验：相机身份尚未核验，请检查相机连接。"),
    ("Recorder exited before collection completed (9)", "采集完成前录制进程已退出（退出码 9）。"),
])
def test_english_calibration_diagnostic_is_localized_in_chinese(raw, chinese):
    original = {"detail": raw, "camera_name": "camera_identity_unverified"}
    displayed = messages.message(original["detail"])
    english = str(displayed)
    assert not any("\u4e00" <= char <= "\u9fff" for char in english)
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == chinese
    assert original == {"detail": raw, "camera_name": "camera_identity_unverified"}
    i18n.set_language("en", persist=False)
    assert str(displayed) == english


def test_english_template_preserves_opaque_paths_and_unknown_nested_errors():
    raw_reason = "Custom failure: '/用户/相机/{v0}/camera_identity_unverified'"
    raw = "Live calibration identity not verified: " + raw_reason
    display = messages.message(raw)
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == "实时标定设备身份尚未核验：" + raw_reason
    i18n.set_language("en", persist=False)
    assert str(display) == raw


@pytest.mark.parametrize("raw", [
    "/data/camera_identity_unverified", "Camera failure: unknown third-party detail",
    "My note: An earlier calibration is still active.",
])
def test_chinese_diagnostic_display_never_translates_unknown_text_partially(raw):
    i18n.set_language("zh_CN", persist=False)
    assert str(messages.message(raw)) == raw


def test_ambiguous_english_translation_is_not_guessed(monkeypatch):
    sources = ("相机连接不可用", "追踪器连接不可用")
    monkeypatch.setattr(messages, "_load_sources", lambda: sources)
    monkeypatch.setattr(messages, "catalog", lambda: dict.fromkeys(sources, "Device unavailable"))
    i18n.set_language("zh_CN", persist=False)
    assert str(messages.message("Device unavailable")) == "Device unavailable"


@pytest.mark.parametrize("domain", ["calibration", "management"])
def test_all_modular_aliases_have_canonical_bilingual_display(domain):
    directory = Path(messages.__file__).parent
    entries = json.loads((directory / f"catalog_diagnostics_{domain}.json").read_text(encoding="utf-8"))
    aliases = json.loads((directory / f"diagnostic_aliases_{domain}.json").read_text(encoding="utf-8"))
    exact, _ = messages._index()
    assert entries.keys() <= exact
    assert set(aliases.values()) <= entries.keys()
    for raw, source in aliases.items():
        display = messages.message(raw)
        assert str(display) == entries[source], raw
        i18n.set_language("zh_CN", persist=False)
        assert str(display) == source, raw
        i18n.set_language("en", persist=False)


@pytest.mark.parametrize("domain", ["calibration", "management"])
def test_every_authored_modular_diagnostic_retranslates_both_ways(domain):
    path = Path(messages.__file__).with_name(f"catalog_diagnostics_{domain}.json")
    for chinese, english in json.loads(path.read_text(encoding="utf-8")).items():
        fields = {field: "42" for _, field, _, _ in Formatter().parse(chinese)
                  if field is not None}
        raw = english.format(**fields)
        display = messages.message(raw)
        i18n.set_language("zh_CN", persist=False)
        assert str(display) == chinese.format(**fields), raw
        i18n.set_language("en", persist=False)
        assert str(display) == raw


def test_english_template_keeps_already_formatted_values_in_chinese():
    raw = "Disk space reached the stop threshold: 1.50 GiB available; 2.28 GiB reserved."
    display = messages.message(raw)
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == "磁盘空间达到停止阈值：可用 1.50 GiB，预留 2.28 GiB。"
    i18n.set_language("en", persist=False)
    assert str(display) == raw


@pytest.mark.parametrize("raw,chinese", [
    ("controller_unavailable", "录制控制器当前不可用，请等待重新连接。"),
    ("state_revision_mismatch", "录制状态已变化，请刷新后重试。"),
    ("queue_full", "操作队列已满，请等待当前操作完成。"),
    ("Recorder process started", "录制已开始。"),
    ("An active or uncertain Recorder session already exists", "已有活动或状态待核实的录制，请先确认当前录制状态。"),
    ("Resources are already reserved by group cameras.", "资源已被设备组 cameras 占用。"),
])
def test_management_diagnostics_are_localized_in_chinese(raw, chinese):
    display = messages.message(raw)
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == chinese


def test_management_error_capture_remains_opaque():
    reason = "Permission denied: /用户/配置/{自定义}/queue_full"
    raw = "Configured process could not be launched: " + reason
    display = messages.message(raw)
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == "配置的设备进程无法启动：" + reason
    i18n.set_language("en", persist=False)
    assert str(display) == raw


def test_all_diagnostic_resources_obey_aggregate_limits():
    sources, templates = messages._index()
    aliases, alias_templates = messages._alias_index(sources)
    assert len(sources) <= messages.MAX_SOURCES
    assert len(templates) <= messages.MAX_TEMPLATES
    assert len(aliases) <= messages.MAX_ALIASES
    assert len(alias_templates) <= messages.MAX_TEMPLATES
    for pattern in ("catalog_diagnostics_*.json", "diagnostic_aliases_*.json"):
        paths = messages._resource_paths(pattern)
        assert len(paths) <= messages.MAX_RESOURCE_FILES
        assert all(path.stat().st_size <= messages.MAX_RESOURCE_BYTES for path in paths)


def test_oversized_diagnostic_resource_is_rejected_before_decoding(tmp_path):
    path = tmp_path / "diagnostic_aliases_oversized.json"
    path.write_bytes(b" " * (messages.MAX_RESOURCE_BYTES + 1))
    with pytest.raises(ValueError, match="resource exceeds its size limit"):
        messages._read_resource(path)


def test_excessive_modular_resource_count_is_rejected(monkeypatch):
    # Restore fault injection before the language fixture refreshes surviving
    # Qt controls during teardown.
    with monkeypatch.context() as patch:
        patch.setattr(messages, "MAX_RESOURCE_FILES", 1)
        with pytest.raises(ValueError, match="Too many diagnostic translation resource files"):
            messages._resource_paths("catalog_diagnostics_*.json")


def test_aggregate_alias_limit_is_enforced(monkeypatch):
    with monkeypatch.context() as patch:
        patch.setattr(messages, "MAX_ALIASES", 1)
        sources, _ = messages._index()
        with pytest.raises(ValueError, match="aliases exceed their limits"):
            messages._alias_index(sources)


def test_no_duplicate_catalog_keys_or_corrupt_diagnostic_json():
    directory = Path(messages.__file__).parent
    keys = set()
    for path in directory.glob("catalog_*.json"):
        entries = json.loads(path.read_text(encoding="utf-8"))
        assert not keys.intersection(entries), path
        keys.update(entries)
    sources = json.loads((directory / "diagnostic_sources.json").read_text(encoding="utf-8"))
    assert len(sources) == len(set(sources))
