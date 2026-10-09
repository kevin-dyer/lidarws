"""Tests for the PointCloud2 decoding used by surface reconstruction."""
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from tilt_coordinator.mesh_postprocessor_node import (
    pointcloud2_to_numpy,
    reconstruct_ball_pivoting,
    reverse_triangle_winding,
)
from tilt_coordinator.topic_names import (
    CAPTURE_COMPLETE_TOPIC,
    POINT_CLOUD_TOPIC,
)


def test_capture_and_point_cloud_topics_are_root_level():
    assert POINT_CLOUD_TOPIC == '/point_cloud'
    assert CAPTURE_COMPLETE_TOPIC == '/capture_complete'


def test_reverse_triangle_winding_flips_saved_and_displayed_mesh_faces():
    triangles = np.array([[2, 5, 9], [0, 1, 3]], dtype=np.int32)

    assert np.array_equal(
        reverse_triangle_winding(triangles),
        np.array([[2, 9, 5], [0, 3, 1]], dtype=np.int32),
    )


def test_reverse_triangle_winding_requires_triangle_indices():
    with pytest.raises(ValueError, match='Nx3 array'):
        reverse_triangle_winding(np.array([[0, 1]]))


def test_reconstruct_ball_pivoting_creates_mesh_from_synthetic_surface():
    phi = np.linspace(0.1, np.pi - 0.1, 12)
    theta = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    points = np.array([
        [np.sin(p) * np.cos(t), np.sin(p) * np.sin(t), np.cos(p)]
        for p in phi for t in theta
    ])

    mesh = reconstruct_ball_pivoting(points)

    assert len(mesh.vertices) > 0
    assert len(mesh.triangles) > 0


def test_pointcloud2_to_numpy_extracts_finite_xyz():
    message = PointCloud2()
    message.height = 1
    message.width = 3
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 12
    message.row_step = 36
    message.data = b''.join([
        struct.pack('<fff', 1.0, 2.0, 3.0),
        struct.pack('<fff', 4.0, float('nan'), 6.0),
        struct.pack('<fff', 7.0, 8.0, 9.0),
    ])

    assert np.array_equal(
        pointcloud2_to_numpy(message),
        np.array([[1.0, 2.0, 3.0], [7.0, 8.0, 9.0]]),
    )


def test_pointcloud2_to_numpy_requires_xyz():
    message = PointCloud2()

    with pytest.raises(ValueError, match='must contain x, y and z'):
        pointcloud2_to_numpy(message)
