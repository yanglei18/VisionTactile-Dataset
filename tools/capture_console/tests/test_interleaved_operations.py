"""Shared command channels must retain each caller's receipt independently."""
from copy import deepcopy

from vt_capture_core import (
    ControllerSnapshot, DeviceControlSnapshot, ManagementSnapshot,
    ProcessControlSnapshot, ProcessGroupSnapshot,
)
from vt_capture_core.recipes import Recipe
from vt_capture_console.backend import ipc_client
from vt_capture_console.backend.ipc_client import IpcBackend
from vt_capture_console.backend.scene_demo import DemoSceneBackend
from vt_capture_console.ui.main_window import MainWindow


def test_v5_device_command_cannot_hide_successful_recording_receipt(qapp, monkeypatch, tmp_path):
    snapshot = DeviceControlSnapshot('live', 'manager',
        management=ManagementSnapshot(enabled=True, controller=ControllerSnapshot('controller')),
        process_control=ProcessControlSnapshot(enabled=True, groups=[ProcessGroupSnapshot(
            'left_tactile', 'Optional tactile', 'tactile', state='STOPPED', can_start=True)])).to_dict()
    snapshot['capabilities'].update(record=True, device_control=True, capture_plan=True)
    snapshot['recording']['state'] = 'IDLE'
    snapshot['management']['controller']['topics'] = list(Recipe('standard').record_topics())
    snapshot['management']['batches'] = [dict(id='batch', label='Batch', created_at='2026-01-01T00:00:00Z')]
    monkeypatch.setattr(IpcBackend, '_run', lambda self: None)
    backend = IpcBackend(tmp_path / 'unused.sock')
    backend._accept_snapshot(deepcopy(snapshot), 5)
    calls = []

    def exchange(path, request, **kwargs):
        calls.append(deepcopy(request))
        value = deepcopy(snapshot)
        value['management']['controller'].update(state=3, recorder_alive=True, session_id='session')
        value['recording'].update(state='RECORDING', session_id='session')
        value['management']['trials'] = [dict(id='trial', batch_id='batch', label='Trial',
            state='RECORDING', session_id='session', output_path='/synthetic/bag', duration_s=30.,
            elapsed_s=1., quality='NOT_ASSESSED', detail='recording', created_at='2026-01-01T00:00:00Z')]
        for group in value['process_control']['groups']:
            group.update(can_start=False, can_stop=False, can_restart=False)
        start = request['command'] == 'capture.start'
        receipt = dict(request_id=request['request_id'], state='accepted' if start else 'rejected',
                       code='STARTED' if start else 'RECORDER_BUSY', detail='Synthetic receipt',
                       trial_id='trial' if start else '')
        value['management']['operation'] = receipt
        return dict(version=5, request_id=request['request_id'], ok=True, snapshot=value, operation=receipt)

    monkeypatch.setattr(ipc_client, '_exchange', exchange)
    window = MainWindow(backend, scene_backend=DemoSceneBackend(), language_override='zh_CN')
    window.timer.stop()
    try:
        window.navigation.button(4).click()
        assert window.start_button.isEnabled()
        window.start_button.click()
        assert window.pending_request
        window.navigation.button(2).click()
        assert window.device_controls.start_button.isEnabled()
        window.device_controls.start_button.click()
        while not backend._commands.empty():
            backend._send_command(backend._commands.get_nowait())
        window.refresh()
        assert [request['command'] for request in calls] == ['capture.start', 'device.start']
        assert all(request['version'] == 5 for request in calls)
        assert 'recipe' in calls[0]['params'] and 'expected_topics_fingerprint' in calls[0]['params']
        assert backend.snapshot()['recording']['state'] == 'RECORDING'
        assert window.pending_request is None
        assert window.stop_button.isEnabled()
        assert window.device_controls.pending_request is None
    finally:
        window.close()


def test_receipts_stay_bounded_preserve_pending_and_report_expiry(monkeypatch, tmp_path):
    from vt_capture_console.backend.operation_receipts import MAX_OPERATION_RECEIPTS
    from test_ipc_client import control_snapshot, operation
    monkeypatch.setattr(IpcBackend, '_run', lambda self: None)
    backend = IpcBackend(tmp_path / 'unused.sock')
    backend._accept_snapshot(control_snapshot(), 2)
    try:
        original = backend.submit('batch.create', {'label': 'Pending'}, 'original')
        for index in range(MAX_OPERATION_RECEIPTS * 2):
            identifier = f'local-{index}'
            backend.submit('batch.create', {'label': 'Other'}, identifier)
            # Complete this request without polling any external state.
            request = backend._commands.get_nowait()
            if request['request_id'] == original:
                request = backend._commands.get_nowait()
            backend._pending.pop(identifier)
            backend._record_operation(operation(identifier))
        assert backend.operation_result(original)['code'] == 'QUEUED'
        assert backend.operation_result('local-0')['code'] == 'RECEIPT_UNAVAILABLE'
        assert len(backend._receipts._values) <= MAX_OPERATION_RECEIPTS
        returned = backend.operation_result(original)
        returned['state'] = 'rejected'
        assert backend.operation_result(original)['state'] == 'unknown'
        observed = control_snapshot()
        observed['management']['operation'] = operation(original)
        backend._accept_snapshot(observed, 2)
        assert backend.operation_result(original)['state'] == 'accepted'
        assert backend.latest_operation['request_id'] == f'local-{MAX_OPERATION_RECEIPTS * 2 - 1}'
    finally:
        backend.close()


def test_queued_direct_reply_cannot_regress_final_snapshot_receipt(monkeypatch, tmp_path):
    from test_ipc_client import process_snapshot
    monkeypatch.setattr(IpcBackend, '_run', lambda self: None)
    backend = IpcBackend(tmp_path / 'unused.sock')
    backend._accept_snapshot(process_snapshot(), 4)
    try:
        first = backend.submit('device.start', {'group_id': 'cameras', 'expected_manager_instance_id': 'manager'})
        backend.submit('batch.create', {'label': 'Concurrent batch'})
        final = dict(request_id=first, state='accepted', code='started', detail='Owned process launched', trial_id='')
        snapshot = process_snapshot()
        snapshot['management']['operation'] = final
        monkeypatch.setattr(ipc_client, '_exchange', lambda *args, **kwargs: dict(
            version=4, request_id=first, ok=True, snapshot=snapshot,
            operation={**final, 'code': 'queued', 'detail': 'Earlier queued receipt'}))
        backend._send_command(backend._commands.get_nowait())
        assert backend.operation_result(first) == final
    finally:
        backend.close()


def test_expired_receipt_remains_readable_in_both_interface_languages():
    from vt_capture_console import i18n
    from vt_capture_console.i18n.messages import message
    from vt_capture_console.backend.operation_receipts import OperationReceipts
    original = i18n.language()
    try:
        receipt = OperationReceipts().result('expired')
        i18n.set_language('zh_CN', persist=False)
        assert '回执' in str(message(receipt['detail']))
        i18n.set_language('en', persist=False)
        assert 'receipt' in str(message(receipt['detail']))
    finally:
        i18n.set_language(original, persist=False)
