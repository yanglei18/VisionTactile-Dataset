"""Exercise the Chinese mounting workflow through the actual Qt controls."""
from types import SimpleNamespace as N
import time

import numpy as np
import pytest

from vt_capture_manager.optical.hand_fusion import read_mounts


@pytest.fixture
def interface(monkeypatch):
    pytest.importorskip('PyQt5')
    pytest.importorskip('rclpy')
    from vt_capture_manager.optical import mount_dialog as ui
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    app = ui.QtWidgets.QApplication.instance() or ui.QtWidgets.QApplication([])
    yield ui
    app.processEvents()


@pytest.mark.parametrize('hand, index_x, little_x', [('left', 145, 115), ('right', 115, 145)])
def test_dialog_freezes_one_frame_refits_one_point_and_persists_side(interface, monkeypatch, tmp_path,
                                                                   hand, index_x, little_x):
    ui = interface
    k = np.array([[200., 0., 120.], [0., 200., 90.], [0., 0., 1.]])
    image = np.zeros((180, 240, 3), np.uint8)
    stamp = [100]
    pixels = [(130, 135), (index_x, 92), (little_x, 92)]

    def measured(xy):
        return np.array([.002*(xy[0]-120), .002*(xy[1]-90), .4])

    class Depth:
        def __init__(self, node, camera):
            self.closed = False

        def freeze(self, tick, camera, distortion, size):
            assert tick == stamp[0] and size == (240, 180)
            return self

        def point_at(self, xy):
            if xy == (5, 5):
                raise ValueError('没有可靠深度')
            return measured(xy)

        def close(self):
            self.closed = True

    joints = np.zeros((21, 3))
    joints[5], joints[17] = [measured(p)-measured(pixels[0]) for p in pixels[1:]]
    monkeypatch.setattr(ui, 'MountDepthCapture', Depth)
    monkeypatch.setattr(ui, 'load_mount_model', lambda path, side: N(joints_m=joints, rest_sha256='b'*64))
    obs = N(translation_m=(.02, .03, .385), quaternion_xyzw=(0, 0, 0, 1), center_px=(130, 105))
    states = {s: N(observation=obs if s == hand else None) for s in ('left', 'right')}

    def frames():
        return [(time.monotonic(), image, k, np.zeros(5), [obs], states, stamp[0])]

    path = tmp_path/'mounts.json'
    node = N(camera_name='d436', layout_sha256='a'*64)
    dialog = ui.MountDialog(None, node, frames, {}, path)
    try:
        dialog.timer.stop()
        dialog.side.setCurrentIndex(dialog.side.findData(hand))
        dialog.freeze_button.click()
        assert '手腕中央' in dialog.note.text()
        dialog.preview.clicked.emit(pixels[0], 100)
        assert '食指指根' in dialog.note.text()
        dialog.preview.clicked.emit((5, 5), 100)
        assert len(dialog.clicks) == 1 and '没有可靠深度' in dialog.note.text()
        dialog.preview.clicked.emit(pixels[1], 100)
        dialog.preview.clicked.emit(pixels[2], 99)
        assert '小指指根' in dialog.note.text() and not dialog.save_button.isEnabled()
        dialog.preview.clicked.emit(pixels[2], 100)
        dialog.undo.click()
        assert len(dialog.clicks) == 2 and not dialog.save_button.isEnabled()
        dialog.preview.clicked.emit(pixels[2], 100)
        stamp[0] += 1
        dialog.refresh()
        assert dialog.preview.source_geometry[-1] == 101 and '绿色骨架' in dialog.note.text()
        assert not path.exists()
        dialog.save_button.click()
        assert dialog.result() == ui.QtWidgets.QDialog.Accepted
        mounts = read_mounts(path)
        assert set(mounts) == {hand} and mounts[hand].model_rest_sha256 == 'b'*64
    finally:
        dialog.finish_capture()
        dialog.close()
    assert dialog.depth.closed


def test_unbound_simultaneous_cubes_cannot_select_a_hand_by_screen_side(interface):
    row = (0, None, None, None, [object(), object()], {s: N(observation=None) for s in ('left', 'right')}, 5)
    with pytest.raises(ValueError, match='先在跟踪窗口绑定'):
        interface.hand_to_freeze(row, 'left')
