import itertools

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from vt_capture_manager.optical.cube import CubeLayout
from vt_capture_manager.optical.layout_learning import CubeLayoutLearner

K = np.array([[850.,0,320],[0,850.,240],[0,0,1.]])
D = np.zeros(5)
# This is a synthetic second layout, not a declaration of the four unseen tags.
TRUTH = CubeLayout(((0, "back", 1), (1, "left", 2), (2, "top", 2),
                    (3, "right", 1), (4, "bottom", 0), (5, "front", 3)))


def view(direction):
    direction = np.asarray(direction, dtype=float)
    outside = direction / np.linalg.norm(direction)
    forward = -outside
    right = np.cross(forward, [0,0,1])
    right /= np.linalg.norm(right)
    rotation = np.array([right, np.cross(forward, right), forward])
    rvec = cv2.Rodrigues(rotation)[0]
    observations = []
    for tag, obj in TRUTH.object_points().items():
        if obj.mean(axis=0) @ outside > .004:
            pixels = cv2.projectPoints(obj, rvec, np.array([0.,0.,.22]), K, D)[0].reshape(4,2)
            observations.append((tag, pixels))
    return observations


def varied_views():
    for xyz in itertools.product((-1,1), repeat=3):
        for sizes in ((.45,.65,1.), (1.,.5,.7), (.6,1.,.55), (1.,1.,1.)):
            yield np.array(xyz) * sizes


def test_recovers_six_face_layout_without_inputting_unseen_ids_and_holds_out_new_views(tmp_path):
    learner = CubeLayoutLearner()
    source = iter(varied_views())
    while len(learner.snapshot()["known_ids"]) < 6:
        learner._observe(view(next(source)), K, D)
    assert learner.layout.sha256 == TRUTH.sha256
    assert not learner.complete
    path = tmp_path / 'cube.json'
    path.write_text('old')
    with pytest.raises(ValueError):
        learner.save(path)
    assert path.read_text() == 'old'
    # The second acquisition passes through newly visible faces and the opposite
    # side, with viewpoint offsets relative to the original discovery views.
    for direction in varied_views():
        learner._observe(view(direction + [.24,.29,.18]), K, D)
    assert learner.complete
    assert learner.snapshot()['verified_faces'] == 6
    learner.save(path)
    assert CubeLayout.from_file(path).sha256 == TRUTH.sha256


def test_no_learning_from_stationary_or_multicube_frames():
    learner = CubeLayoutLearner()
    sightings = view((.8,-1.,1.))
    for _ in range(9):
        learner._observe(sightings, K, D)
    assert learner.snapshot()['known_ids'] == [2,5]
    learner._observe(sightings + sightings[:1], K, D)
    assert learner.snapshot()['status'] == 'multiple_cubes'
    assert not learner.complete


def test_inconsistent_layout_cannot_overwrite_previous_file(tmp_path):
    swapped = CubeLayout(tuple((tag, face, (corner+1)%4 if tag==3 else corner) for tag,face,corner in TRUTH.faces))
    learner = CubeLayoutLearner(swapped)
    learner._observe(view((.8,-1.,1.)), K, D)
    assert learner.snapshot()['status'] == 'layout_conflict'
    path = tmp_path/'layout.json'
    path.write_text('unchanged')
    with pytest.raises(ValueError):
        learner.save(path)
    assert path.read_text() == 'unchanged'


def test_update_decodes_image_once_and_preserves_partial_model():
    learner = CubeLayoutLearner()
    blank = np.zeros((480,640), dtype=np.uint8)
    assert learner.update(blank, K, D)['status'] == 'show_adjacent_tags'
    assert learner.layout.sha256 == CubeLayout.default().sha256


def render_sightings(sightings):
    image = np.full((480,640), 255, dtype=np.uint8)
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    for tag, pixels in sightings:
        if hasattr(aruco, "generateImageMarker"):
            marker = aruco.generateImageMarker(dictionary, tag, 96)
        else:
            marker = aruco.drawMarker(dictionary, tag, 96)
        src = np.float32([[0,0], [95,0], [95,95], [0,95]])
        transform = cv2.getPerspectiveTransform(src, pixels.astype('float32'))
        image = np.minimum(image, cv2.warpPerspective(marker, transform, (640,480), borderValue=255))
    return image


def test_six_face_scan_from_decoded_pixels():
    learner = CubeLayoutLearner()
    for offset in ([0,0,0], [.24,.29,.18], [-.32,.22,-.15]):
        for direction in varied_views():
            frame = render_sightings(view(direction+offset))
            progress = learner.update(frame, K, D)
            assert progress["status"] != "layout_conflict"
            if progress["complete"]:
                assert learner.layout.sha256 == TRUTH.sha256
                return
    pytest.fail(str(learner.snapshot()))
