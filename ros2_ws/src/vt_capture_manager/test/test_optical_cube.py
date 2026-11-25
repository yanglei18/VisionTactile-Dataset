"""Synthetic photographs of identical printed tags on two hand-sized cubes."""
import json

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from vt_capture_manager.optical.cube import CubeLayout, CubeDetector

K = np.array([[840., 0, 480], [0, 840., 300], [0, 0, 1.]])
D = np.zeros(5)
# Independent hand-entered corner order from the two printed faces.
P = {
    2: np.array([[12,-12,15], [-12,-12,15], [-12,12,15], [12,12,15]]) / 1000.,
    5: np.array([[-12,-15,-12], [-12,-15,12], [12,-15,12], [12,-15,-12]]) / 1000.,
}


def looking_down(angle):
    """Camera looking towards the front of the cube and down onto the top."""
    a = np.deg2rad(angle)
    return cv2.Rodrigues(np.array([[1.,0,0], [0,-np.sin(a),-np.cos(a)],
                                  [0,np.cos(a),-np.sin(a)]]))[0]


def image_for(poses):
    gray = np.full((600, 960), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    src = np.float32([[0,0], [159,0], [159,159], [0,159]])
    for tvec, angle, tags in poses:
        for tag in tags:
            uv = cv2.projectPoints(P[tag], looking_down(angle), np.array(tvec), K, D)[0].reshape(4, 2)
            if hasattr(cv2.aruco, "generateImageMarker"):
                marker = cv2.aruco.generateImageMarker(dictionary, tag, 160)
            else:
                marker = cv2.aruco.drawMarker(dictionary, tag, 160)
            transform = cv2.getPerspectiveTransform(src, uv.astype('float32'))
            gray = np.minimum(gray, cv2.warpPerspective(marker, transform, (960,600), borderValue=255))
    return gray


def test_two_identical_cubes_cannot_merge_or_overwrite_duplicate_tags():
    detector = CubeDetector(CubeLayout.default())
    expected = [(-.050, .01, .34), (.045, -.005, .32)]
    observed = detector.detect(image_for([(expected[0], 33., (2, 5)), (expected[1], 55., (2, 5))]), K, D)
    assert len(observed) == 2
    for observation, target, angle in zip(sorted(observed, key=lambda o: o.center_px[0]), expected, [33.,55.]):
        assert observation.tag_ids == (2,5)
        np.testing.assert_allclose(observation.translation_m, target, atol=.003)
        assert observation.reprojection_px < 1.25
        rotation = looking_down(angle).reshape(3)
        q = np.r_[rotation/np.linalg.norm(rotation)*np.sin(np.linalg.norm(rotation)/2), np.cos(np.linalg.norm(rotation)/2)]
        assert abs(q @ observation.quaternion_xyzw) > np.cos(np.deg2rad(3)/2)


def test_same_id_alone_is_a_sighting_per_cube_not_a_hand_identity():
    detector = CubeDetector(CubeLayout.default())
    objects = [((-.06, 0., .28), 53., (2,)), ((.055, 0., .28), 40., (2,))]
    result = detector.detect(image_for(objects), K, D)
    assert len(result) == 2
    assert all(o.tag_ids == (2,) for o in result)


def test_ambiguous_planar_solutions_suppressed_and_no_ghost_in_blank_image():
    detector = CubeDetector(CubeLayout.default())
    # Too small to select one of the distinct shallow-angle single-face poses
    # from pixel error alone, even when exact noiseless corners are available.
    rvec = looking_down(65.)
    points = cv2.projectPoints(P[2], rvec, np.array([0.,0.,.85]), K, D)[0]
    detector._detect = lambda gray: ([points], np.array([[2]]), [])
    assert detector.detect(np.zeros((100,100),np.uint8), K, D) == []
    assert CubeDetector(CubeLayout.default()).detect(np.zeros((100,100),np.uint8), K, D) == []


def test_layout_points_and_file_validation(tmp_path):
    doc = dict(family='tag36h11', cube_edge_m=.030, tag_edge_m=.024,
               faces=[dict(id=5, face='front', corner0=3), dict(id=2, face='top', corner0=2)])
    p = tmp_path/'layout.json'
    p.write_text(json.dumps(doc))
    restored = CubeLayout.from_file(p)
    assert restored.sha256 == CubeLayout.default().sha256
    for tag in (2,5):
        np.testing.assert_allclose(restored.object_points()[tag], P[tag], atol=1e-15)
    doc['faces'].append(dict(id=4, face='right', corner0=0))
    p.write_text(json.dumps(doc))
    assert CubeLayout.from_file(p).sha256 != restored.sha256
    for invalid in [dict(id=2,face='left',corner0=0), dict(id=1,face='right',corner0=0),
                    dict(id=3,face='left',corner0=4), dict(id=0,face='left',corner0=True)]:
        p.write_text(json.dumps(dict(doc, faces=doc['faces']+[invalid])))
        with pytest.raises(ValueError):
            CubeLayout.from_file(p)


def test_two_faces_of_one_cube_form_one_measurement_and_corner_winding_matches():
    rgb = cv2.cvtColor(image_for([((0.,0.,.27),42.,(2,5))]),cv2.COLOR_GRAY2BGR)
    obs, = CubeDetector(CubeLayout.default()).detect(rgb, K, D)
    assert obs.tag_ids == (2,5)
    assert np.linalg.norm(np.asarray(obs.translation_m)-[0.,0.,.27]) < .003


def test_layout_save_creates_or_explicitly_replaces_and_cleans_up(tmp_path):
    path = tmp_path / 'config' / 'cube.json'
    original = CubeLayout.default()
    changed = CubeLayout((*original.faces, (1, 'back', 0)))
    original.save(path)
    with pytest.raises(FileExistsError):
        changed.save(path)
    assert CubeLayout.from_file(path).sha256 == original.sha256
    changed.save(path, replace=True)
    assert CubeLayout.from_file(path).sha256 == changed.sha256
    assert list(path.parent.iterdir()) == [path]
