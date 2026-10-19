import math

import pytest

from vt_capture_core.spatial.transforms import RigidTransform


def test_composition_and_inverse_preserve_coordinate_direction():
    parent = RigidTransform((1, 2, 3), (0, 0, math.sqrt(.5), math.sqrt(.5)))
    child = RigidTransform((2, 0, 0), (0, 0, 0, 1))
    assert parent.compose(child).apply((0, 0, 0)) == pytest.approx((1, 4, 3))
    point = (.2, -.4, 9)
    assert parent.inverse().apply(parent.apply(point)) == pytest.approx(point)
    assert parent.compose(parent.inverse()).translation_m == pytest.approx((0, 0, 0))


def test_interpolation_shortest_arc_and_negative_quaternion():
    first = RigidTransform.identity()
    second = RigidTransform((2, 0, 0), (0, 0, -math.sqrt(.5), -math.sqrt(.5)))
    middle = first.interpolate(second, .5)
    assert middle.translation_m == (1, 0, 0)
    assert middle.apply((1, 0, 0)) == pytest.approx((1 + math.sqrt(.5), math.sqrt(.5), 0))
    assert first.interpolate(RigidTransform((0, 0, 0), (0, 0, 0, -1)), .5).apply((1, 2, 3)) == pytest.approx((1, 2, 3))


@pytest.mark.parametrize('translation,quaternion', [
    ((float('nan'), 0, 0), (0, 0, 0, 1)), ((0, 0, 0), (0, 0, 0, 0)),
    ((0, 0, 0), (0, 0, 0, 2)), ((True, 0, 0), (0, 0, 0, 1)),
])
def test_invalid_transform_rejected(translation, quaternion):
    with pytest.raises(ValueError):
        RigidTransform(translation, quaternion)


def test_transform_owns_tuples_and_roundtrips():
    translation = [1, 2, 3]
    pose = RigidTransform(translation, [0, 0, 0, 1])
    translation[0] = 90
    assert pose.translation_m == (1, 2, 3)
    assert RigidTransform.from_dict(pose.as_dict()) == pose
    for fraction in (-1, 2, float('nan')):
        with pytest.raises(ValueError):
            pose.interpolate(pose, fraction)


def test_math_agrees_with_existing_offline_implementation():
    offline = pytest.importorskip('vt_multisensor_alignment.model')
    import random
    randomizer = random.Random(813)
    for _ in range(30):
        transforms = []
        for _ in range(2):
            axis = [randomizer.uniform(-1, 1) for _ in range(3)]
            norm = math.sqrt(sum(value * value for value in axis))
            angle = randomizer.uniform(-math.pi, math.pi) / 2
            q = tuple(value / norm * math.sin(angle) for value in axis) + (math.cos(angle),)
            transforms.append(RigidTransform([randomizer.random() for _ in range(3)], q))
        a, b = transforms
        old_a = offline.Transform(a.translation_m, a.quaternion_xyzw)
        old_b = offline.Transform(b.translation_m, b.quaternion_xyzw)
        for current, old in [(a.compose(b), old_a.compose(old_b)), (a.interpolate(b, .37), old_a.interpolate(old_b, .37))]:
            assert current.translation_m == pytest.approx(old.translation, abs=1e-12)
            assert current.quaternion_xyzw == pytest.approx(old.quaternion_xyzw, abs=1e-12)
