"""Real worker entry-point smoke; opt in with its isolated Python interpreter."""
import os
from pathlib import Path
import time

import pytest

from vt_capture_manager.calibration.config import build_config
from vt_capture_manager.calibration.service import CalibrationService
from vt_capture_manager.calibration.storage import read_document

pytestmark = pytest.mark.skipif(not os.environ.get('VT_CALIBRATION_WORKER_PYTHON'),
                                reason='separately installed calibration worker interpreter required')


@pytest.mark.parametrize('cancel', [False, True])
def test_real_worker_subprocess_uses_private_contract_and_closes(tmp_path, cancel):
    board = dict(squares_x=9, squares_y=6, square_length_mm=40., marker_length_mm=30., dictionary='DICT_5X5_1000')
    config = build_config(dict(name='cam', model='D405', serial='123'), 'torso', board)
    service = CalibrationService(tmp_path / 'sessions', 'test-manager', os.environ['VT_CALIBRATION_WORKER_PYTHON'],
        can_start=lambda: True, prepare_config=lambda params: dict(config=config, expected_tracker_id=''))
    try:
        result = service.command('calibration.start', dict(expected_manager_instance_id='test-manager', mode='offline',
            camera_name='cam', tracker_role='torso', config_path='', board=board,
            bag_paths=[str(tmp_path / 'missing-bag')]), 'start-real-process')
        assert result['accepted']
        if cancel:
            assert service.command('calibration.cancel', dict(expected_manager_instance_id='test-manager',
                session_id=result['session_id']), 'cancel-real-process')['accepted']
        deadline = time.monotonic() + 15.
        while service.busy and time.monotonic() < deadline:
            service.tick()
            time.sleep(.02)
        assert not service.busy
        snapshot = service.snapshot()
        assert snapshot['session']['state'] == ('CANCELLED' if cancel else 'FAILED')
        status = read_document(tmp_path / 'sessions' / 'sessions' / result['session_id'] / 'status.json')
        assert status['session_id'] == result['session_id']
        assert status['state'] == snapshot['session']['state']
        if not cancel:
            assert 'metadata' in snapshot['session']['detail']
        assert not snapshot['active']
    finally:
        service.close()
