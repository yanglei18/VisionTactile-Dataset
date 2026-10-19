"""Offscreen input-scale UI contract; no reader, hardware, fitter or GL loop."""
from copy import deepcopy

import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('pyqtgraph')
pytest.importorskip('OpenGL')

from vt_fsglove12 import live_ui
from vt_fsglove12.input_contract import describe_input_contract
from test_live_ui import official_window, make_measured


def completed_check(tmp_path):
    return dict(status='completed', message='本次检查完成；6号证据不足，保留原系数。',
        supported_nodes=['1', '5'], recommended_nodes=['5'], inconclusive_nodes=['6'],
        candidate_available=True, report_path=str(tmp_path / 'input-scale-report.json'),
        candidate_path=str(tmp_path / 'input-scale-candidate.json'), config_sha256='checked-config-sha',
        nodes={'5': dict(status='recommended', nominal_gyro_dps_per_lsb=.00875,
                        candidate_gyro_dps_per_lsb=.0175, reasons=['两个独立区间支持候选']),
               '6': dict(status='inconclusive', nominal_gyro_dps_per_lsb=.00875,
                         reason='严格静止检查未通过', reason_codes=['static_hold_unqualified'])})


def install_check(controller, check):
    make_measured(controller)
    controller.state['pose']['mano_pose_rotvec'] = [[0., 0., 0.]] * 16
    controller.state['runtime']['input_scale_generation'] = 0
    controller.state['calibration'].update(input_scale_check=check, profile_status='fitted')


@pytest.mark.parametrize('status,title', [
    ('waiting', '等待三姿态采集'), ('collecting', '采集中'), ('running', '检查中'),
    ('completed', '检查完成'), ('failed', '检查未完成'), ('applied', '候选已采用'),
])
def test_scale_panel_reports_backend_state_without_auto_commands(official_window, tmp_path, status, title):
    window, controller = official_window
    check = {**completed_check(tmp_path), 'status': status}
    install_check(controller, check)
    original = deepcopy(controller.state)
    window.refresh()
    assert title in window.input_scale_summary.text()
    assert window.input_scale_apply_button.isEnabled() == (status == 'completed')
    assert '无需手长、尺子或精确角度' in window.input_scale_guidance.text()
    assert '20 秒' in window.input_scale_guidance.text()
    assert '不要只绕竖直轴' in window.input_scale_guidance.text()
    assert not window.input_scale_panel.isHidden()
    assert window.view.mesh.visible()  # Metadata does not replace or hide pose.
    assert controller.commands == [] and controller.state == original


@pytest.mark.parametrize('active,profile_status,available', [
    (dict(kind='mounting', step='neutral'), 'fitted', True),
    (None, 'fitting', True), (None, 'fitting_functional_hand', True),
    (None, 'clearing', True), (None, 'fitted', False),
])
def test_apply_rechecks_latest_busy_state_even_before_timer_refresh(
        official_window, tmp_path, active, profile_status, available):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    window.refresh()
    assert window.input_scale_apply_button.isEnabled()
    controller.state['calibration'].update(active=active, profile_status=profile_status)
    controller.state['calibration']['input_scale_check']['candidate_available'] = available
    window.input_scale_apply_button.click()
    assert controller.commands == []
    window.refresh()
    assert not window.input_scale_apply_button.isEnabled()


def test_adopt_is_one_explicit_command_waits_for_ack_and_never_connects(official_window, tmp_path):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    original_config = deepcopy(window.config)
    window.refresh()
    window.input_scale_apply_button.click()
    assert controller.commands == [('use_input_scale_candidate', {})]
    assert not window.input_scale_apply_button.isEnabled()
    for _ in range(3):
        window.refresh()
        window.use_input_scale_candidate()
    assert controller.commands == [('use_input_scale_candidate', {})]
    # Worker acknowledgement, not a proxy config or button click, decides the
    # transition. The UI never disconnects/reconnects on its own.
    controller.state['runtime'].update(input_scale_generation=1, connected=False)
    controller.state['calibration']['input_scale_check']['status'] = 'applied'
    window.refresh()
    assert '重新连接并标定' in window.input_scale_summary.text()
    assert window.config == original_config
    assert controller.commands == [('use_input_scale_candidate', {})]


def test_apply_queue_rejection_and_worker_failure_do_not_install_candidate(official_window, tmp_path, monkeypatch):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    original_contract = deepcopy(window.input_contract)
    window.refresh()
    with monkeypatch.context() as patch:
        patch.setattr(controller, 'submit', lambda *_args, **_kwargs: False)
        window.input_scale_apply_button.click()
        assert window._input_scale_apply_pending_generation is None
    window.input_scale_apply_button.click()
    assert not window.input_scale_apply_button.isEnabled()
    controller.state['runtime']['input_scale_generation'] = 1
    controller.state['calibration']['input_scale_check'].update(status='failed', message='候选验证失败；旧配置保留')
    window.refresh()
    assert '旧配置保留' in window.input_scale_summary.text()
    assert window.input_contract == original_contract
    assert not window.input_scale_apply_button.isEnabled()


def test_child_queue_rejection_ack_unlatches_ui_without_execution_or_automatic_retry(official_window, tmp_path):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    previous = deepcopy(window.input_contract)
    window.refresh()
    window.input_scale_apply_button.click()
    assert not window.input_scale_apply_button.isEnabled()
    error = '操作队列繁忙或命令不可用：use_input_scale_candidate；未自动重试。'
    controller.state['runtime'].update(input_scale_rejected_count=1,
        input_scale_command_error=error, error=error)
    window.refresh()
    assert controller.state['runtime']['input_scale_generation'] == 0
    assert window.input_scale_apply_button.isEnabled()
    assert error in window.input_scale_summary.text() and error in window.status.text()
    assert '正在请求采用' not in window.input_scale_summary.text()
    assert controller.commands == [('use_input_scale_candidate', {})]
    assert window.input_contract == previous
    # A second user click tracks the new total; the previous rejection alone
    # must not be mistaken for acknowledgement of this second command.
    window.input_scale_apply_button.click()
    window.refresh()
    assert not window.input_scale_apply_button.isEnabled()
    assert len(controller.commands) == 2
    controller.state['runtime'].update(input_scale_generation=1, input_scale_command_error=None, error=None)
    controller.state['calibration']['input_scale_check']['status'] = 'applied'
    window.refresh()
    assert window._input_scale_apply_pending_generation is None
    assert '后台反馈' not in window.input_scale_summary.text()


def test_empty_calibration_keeps_new_scale_results_but_clear_pending_hides_old(official_window, tmp_path):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    calibration = controller.state['calibration']
    calibration.update(profile_status='not_calibrated', captured_steps=[], calibrated_nodes=[])
    window.refresh()
    assert '检查完成' in window.input_scale_summary.text()
    assert window.input_scale_apply_button.isEnabled()
    window.show_input_scale_check()
    assert 'checked-config-sha' in window.input_scale_details.toPlainText()
    original = deepcopy(controller.state)
    window.clear_calibration()
    assert controller.commands == [('clear_calibration', {})]
    assert window._clear_pending
    assert '检查完成' not in window.input_scale_summary.text()
    assert 'checked-config-sha' not in window.input_scale_details.toPlainText()
    assert not window.input_scale_apply_button.isEnabled()
    assert not window.input_scale_report_button.isEnabled()
    assert controller.state == original
    # The acknowledged new empty state can carry an applied scale result;
    # rebuilding the display's empty calibration must retain it.
    calibration['clear_generation'] = 1
    calibration['input_scale_check'] = dict(status='applied', message='已采用，请重新连接标定')
    window.refresh()
    assert not window._clear_pending
    assert '候选已采用' in window.input_scale_summary.text()


def test_details_and_report_directory_are_read_only(official_window, tmp_path, monkeypatch):
    window, controller = official_window
    check = completed_check(tmp_path)
    install_check(controller, check)
    original = deepcopy(controller.state)
    window.refresh()
    window.input_scale_details_button.click()
    text = window.input_scale_details.toPlainText()
    assert window.input_scale_details.isReadOnly()
    for value in ('节点 5', '0.0175', '节点 6', '严格静止检查未通过', '物理量程读回', check['report_path']):
        assert value in text
    human_readable = text.split('后台结果（只读）：')[0]
    assert '原因：严格静止检查未通过' in human_readable
    assert 'static_hold_unqualified' in human_readable
    opened = []
    monkeypatch.setattr(live_ui.QtGui.QDesktopServices, 'openUrl', lambda url: opened.append(url.toLocalFile()))
    window.input_scale_report_button.click()
    assert opened == [str(tmp_path)]
    assert controller.commands == [] and controller.state == original
    window.close()
    assert not window.input_scale_dialog.isVisible()


@pytest.mark.parametrize('runtime_contract', [True, False])
def test_applied_contract_updates_main_and_open_twelve_node_dialog_not_proxy_config(
        official_window, tmp_path, runtime_contract):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    controller.config = deepcopy(window.config)  # Deliberately obsolete proxy configuration.
    original_config = deepcopy(window.config)
    window.show_input_contract()
    assert window.input_contract_table.item(4, 2).text() == '0.00875'
    changed = deepcopy(original_config)
    changed['gyro_dps_per_lsb_by_node'] = {'5': .0175, '6': .0175}
    # Use the actual config override field, not a hand-built contract.
    contract = describe_input_contract(changed)
    assert contract['nodes']['5']['gyro_dps_per_lsb'] == .0175
    check = controller.state['calibration']['input_scale_check']
    check.update(status='applied', applied_input_contract=deepcopy(contract))
    if runtime_contract:
        controller.state['runtime']['active_input_contract'] = deepcopy(contract)
        # The committed worker contract wins over an older applied-result
        # fallback attached to calibration, as well as proxy configuration.
        check['applied_input_contract'] = describe_input_contract(original_config)
    original = deepcopy(controller.state)
    window.refresh()
    assert '5号=0.0175' in window.details.toPlainText() and '6号=0.0175' in window.details.toPlainText()
    assert window.input_contract_table.item(4, 2).text() == '0.0175'
    assert window.input_contract_table.item(5, 2).text() == '0.0175'
    assert window.input_contract_summary_label.text() == live_ui.input_contract_summary(contract)
    assert window.input_contract == contract and window.config == controller.config == original_config
    assert controller.state == original and controller.commands == []
    # A subsequent sparse snapshot must not replace committed values with the
    # unchanged parent-side controller.config object.
    controller.state['runtime'].pop('active_input_contract', None)
    controller.state['calibration']['input_scale_check'] = {'status': 'waiting'}
    window.refresh()
    assert window.input_contract == contract


def test_unchanged_detail_result_is_not_reserialized_on_every_paint_tick(official_window, tmp_path, monkeypatch):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    window.show_input_scale_check()
    original = live_ui.input_scale_check_text
    detailed_calls = []

    def counted(check, *, detailed=False):
        if detailed:
            detailed_calls.append(check.get('status'))
        return original(check, detailed=detailed)

    monkeypatch.setattr(live_ui, 'input_scale_check_text', counted)
    for _ in range(4):
        window.refresh()
    assert detailed_calls == []
    controller.state['calibration']['input_scale_check']['message'] = '后台结果已更新'
    window.refresh()
    assert detailed_calls == ['completed']
    assert '后台结果已更新' in window.input_scale_details.toPlainText()


def test_scale_panel_preserves_existing_minimum_height_and_scrollable_full_hand_controls(official_window, tmp_path):
    window, controller = official_window
    install_check(controller, completed_check(tmp_path))
    window.refresh()
    window.ensurePolished()
    window.resize(1220, 900)
    window.layout().activate()
    window.centralWidget().layout().activate()
    scroll = window.sidebar_scroll
    scroll.widget().layout().activate()
    with_panel = window.minimumSizeHint().height()
    window.input_scale_panel.hide()
    scroll.widget().layout().activate()
    window.centralWidget().layout().activate()
    without_panel = window.minimumSizeHint().height()
    window.input_scale_panel.show()
    scroll.widget().layout().activate()
    window.centralWidget().layout().activate()
    assert with_panel == without_panel
    # The existing right-side view currently needs about 975 px. At a 900 px
    # request, the new panel must not raise that pre-existing minimum further.
    assert window.height() <= max(900, without_panel)
    assert scroll.widgetResizable() and scroll.verticalScrollBar().maximum() > 0
    # All new and later controls remain inside the vertical scroll area, not
    # outside its available region. No real window or GL paint is started.
    for widget in (window.input_scale_apply_button, window.functional_hand_capture,
                   window.functional_capture, window.clear, window.details):
        assert scroll.isAncestorOf(widget)
    scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum())
    assert scroll.verticalScrollBar().value() > 0
    assert controller.commands == []
