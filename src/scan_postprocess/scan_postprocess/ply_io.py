"""Point cloud file helpers shared by the post-processing tools."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d


@dataclass
class Cloud:
    points: np.ndarray  # (N, 3) float64
    colors: Optional[np.ndarray] = None  # (N, 3) uint8, or None when the file has no colour

    def __len__(self) -> int:
        return len(self.points)


def read_cloud(path: str) -> Cloud:
    pcd = o3d.io.read_point_cloud(path)
    points = np.asarray(pcd.points, dtype=np.float64)
    if len(points) == 0:
        raise ValueError(f'{path}: no points could be read.')
    finite = np.isfinite(points).all(axis=1)
    colors = None
    if pcd.has_colors():
        colors = np.clip(np.asarray(pcd.colors) * 255.0 + 0.5, 0, 255).astype(np.uint8)[finite]
    return Cloud(points[finite], colors)


def write_cloud(path: str, cloud: Cloud, ascii_format: bool = False) -> None:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.points)
    if cloud.colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(cloud.colors.astype(np.float64) / 255.0)
    if not o3d.io.write_point_cloud(path, pcd, write_ascii=ascii_format):
        raise IOError(f'Could not write {path}')
