import numpy as np
import pytest

from scan_postprocess.colorize import colorize_points
from scan_postprocess.ply_io import Cloud, read_cloud, write_cloud
from scan_postprocess.registration import RegistrationError, RegistrationParams, register_clouds


def make_room(n=40000, seed=0):
    """Box room with furniture-like blocks so the geometry is not symmetric."""
    rng = np.random.default_rng(seed)
    pts = []
    for axis, value in ((0, 5.0), (0, -4.0), (1, 3.0), (1, -3.5), (2, 2.5), (2, -0.5)):
        p = rng.uniform([-4, -3.5, -0.5], [5, 3, 2.5], (n // 6, 3))
        p[:, axis] = value
        pts.append(p)
    box = rng.uniform([1, -1, -0.5], [2.5, 0.5, 1.0], (n // 6, 3))
    face = rng.integers(0, 3, len(box))
    box[np.arange(len(box)), face] = np.where(rng.random(len(box)) < 0.5, 1.0, 2.5) * (face == 0) + \
        np.where(rng.random(len(box)) < 0.5, -1.0, 0.5) * (face == 1) + np.where(rng.random(len(box)) < 0.5, -0.5, 1.0) * (face == 2)
    pts.append(box)
    return np.vstack(pts)


def rigid(yaw, shift):
    c, s = np.cos(yaw), np.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    t[:3, 3] = shift
    return t


def test_colorize_assigns_colours_and_skips_occluded():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:, :80] = (200, 20, 20)
    image[:, 80:] = (20, 20, 200)
    front = np.array([[5.0, -0.5, 0.0], [5.0, 0.5, 0.0]])  # y<0 left half, y>0 right half
    hidden = np.array([[10.0, -1.0, 0.0]])  # directly behind the first point
    behind = np.array([[-3.0, 0.0, 0.0]])
    pts = np.vstack([front, hidden, behind])
    result = colorize_points(pts, image, frame='frd')
    assert result.confident.tolist() == [True, True, False, False]
    assert result.colors[0].tolist() == [200, 20, 20]
    assert result.colors[1].tolist() == [20, 20, 200]


def test_colorize_edges_not_confident():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:, 80:] = 255
    result = colorize_points(np.array([[5.0, 0.0, 0.0]]), image)
    assert not result.confident[0]


def test_ply_roundtrip(tmp_path):
    cloud = Cloud(np.random.rand(10, 3), np.random.randint(0, 255, (10, 3), dtype=np.uint8))
    write_cloud(str(tmp_path / 'a.ply'), cloud)
    back = read_cloud(str(tmp_path / 'a.ply'))
    assert np.allclose(back.points, cloud.points)
    assert np.abs(back.colors.astype(int) - cloud.colors).max() <= 1


def test_registration_recovers_transform():
    room = make_room()
    truth = rigid(np.radians(25), [0.6, -0.4, 0.05])
    a = room[room[:, 0] < 3.5]
    b = room[room[:, 1] > -2.5]
    b = b @ truth[:3, :3].T + truth[:3, 3]  # b seen from another place
    colors_a = np.full((len(a), 3), 200, dtype=np.uint8)
    result = register_clouds([Cloud(a, colors_a), Cloud(b)], RegistrationParams(voxel=0.08))
    recovered = result.transformations[1] @ truth
    assert np.linalg.norm(recovered[:3, 3]) < 0.05
    assert np.abs(recovered[:3, :3] - np.eye(3)).max() < 0.02
    assert result.cloud.colors is not None and len(result.cloud.colors) == len(result.cloud.points)


def test_registration_fails_on_unrelated_clouds():
    rng = np.random.default_rng(1)
    a = make_room()
    b = rng.normal(size=(20000, 3)) * 2
    with pytest.raises(RegistrationError):
        register_clouds([Cloud(a), Cloud(b)], RegistrationParams(voxel=0.1))
