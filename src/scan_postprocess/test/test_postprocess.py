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


def test_camera_pose_shifts_projection():
    from scan_postprocess.calibration import CameraModel, rotation_from_degrees
    pts = np.array([[5.0, 0.0, 0.0]])
    cam = CameraModel.nominal(400, 300, 65.0)
    u0, v0, _ = cam.project(pts)
    cam.rotation = cam.rotation @ rotation_from_degrees(0, 10, 0)
    u1, _, _ = cam.project(pts)
    assert u0[0] == pytest.approx(200, abs=1) and abs(u1[0] - u0[0]) > 20


def test_align_rotates_vector_onto_target():
    from scan_postprocess.feature_calibration import _align
    u = np.array([1.0, 0, 0])
    v = np.array([0.0, 0.6, 0.8])
    assert np.allclose(_align(u, v) @ u, v)
    assert np.allclose(_align(u, -u) @ u, -u)


def test_blend_colours_points_seen_by_two_photos():
    import numpy as np
    from scan_postprocess.blend import blend_colors
    from scan_postprocess.calibration import CameraModel
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(4, 5, 400), rng.uniform(-1, 1, 400), rng.uniform(-1, 1, 400)])
    image = np.full((120, 160, 3), 200, np.uint8)
    cam = CameraModel.nominal(160, 120, 90.0, 'frd')
    eye = np.eye(4)
    res = blend_colors(pts, np.zeros(400, int), [image, image], [cam, cam], [eye, eye])
    assert res.confident.any()
    assert (res.colors[res.confident] == 200).all()
    assert res.views.max() == 2


def make_outdoor(seed=0):
    """Ground, a long wall and a few posts/boxes: the wall alone would let scans slide along it."""
    rng = np.random.default_rng(seed)
    ground = np.c_[rng.uniform(-10, 10, (30000, 2)), rng.normal(0, 0.01, 30000)]
    wall = np.c_[rng.uniform(-10, 10, 12000), np.full(12000, 6.0), rng.uniform(0, 2.5, 12000)]
    parts = [ground, wall]
    for cx, cy, w, h in ((0, 2, 0.5, 1.5), (3, -2, 0.8, 1.0), (-4, 3, 0.4, 2.0), (5, 3, 1.0, 0.7), (-2, -3, 0.6, 1.2)):
        for _ in range(3):
            face = rng.uniform(-w, w, (700, 3)) * [1, 1, 0] + [cx, cy, 0]
            face[:, 2] = rng.uniform(0, h, 700)
            side = rng.integers(0, 2, 700)
            face[:, side[0]] = np.where(rng.random(700) < 0.5, 1, -1) * w + [cx, cy][side[0]]
            parts.append(face)
    return np.vstack(parts)


def scan_from(scene, yaw, position, radius=9.0):
    """Scene as a scanner 1 m above the ground at `position` facing `yaw` would record it (noisy, limited range)."""
    pose = rigid(yaw, [position[0], position[1], 1.0])
    local = (scene - pose[:3, 3]) @ pose[:3, :3]
    local = local[np.linalg.norm(local, axis=1) < radius]
    return local + np.random.default_rng(3).normal(0, 0.01, local.shape), pose


def test_ground_registration_outdoor():
    scene = make_outdoor()
    scans = [scan_from(scene, 0.2, (-3, -4)), scan_from(scene, 0.9, (2, -5)), scan_from(scene, 1.5, (-6, 0))]
    result = register_clouds([Cloud(p) for p, _ in scans], RegistrationParams(voxel=0.1))
    assert result.excluded == []
    # Relative poses between scans must match the true ones (planar error well under the wall slide ambiguity).
    for k in (1, 2):
        truth = np.linalg.inv(scans[0][1]) @ scans[k][1]
        error = np.linalg.inv(result.transformations[k]) @ truth
        assert np.hypot(error[0, 3], error[1, 3]) < 0.15
        assert abs(np.arctan2(error[1, 0], error[0, 0])) < np.radians(2)


def test_ground_registration_leaves_out_unrelated_scan():
    scene = make_outdoor()
    rng = np.random.default_rng(5)
    stray = np.c_[rng.uniform(-10, 10, (20000, 2)), rng.normal(0, 0.01, 20000)] - [0, 0, 1.0]
    stray = np.vstack([stray, rng.normal(size=(500, 3)) * [3, 3, 0.3] + [0, 0, 0.4]])
    scans = [scan_from(scene, 0.2, (-3, -4)), scan_from(scene, 0.9, (2, -5))]
    result = register_clouds([Cloud(scans[0][0]), Cloud(stray), Cloud(scans[1][0])], RegistrationParams(voxel=0.1))
    assert result.included == [0, 2] and result.excluded == [1]
