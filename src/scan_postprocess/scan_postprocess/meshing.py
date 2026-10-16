"""Surface reconstruction (Poisson) from a registered, optionally coloured point cloud."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .ply_io import Cloud


@dataclass
class Mesh:
    vertices: np.ndarray  # (V, 3)
    triangles: np.ndarray  # (F, 3) int
    colors: Optional[np.ndarray] = None  # (V, 3) uint8


def build_mesh(
    cloud: Cloud,
    source_index: Optional[np.ndarray] = None,
    origins: Optional[Sequence[np.ndarray]] = None,
    voxel: float = 0.03,
    depth: int = 9,
    trim_density: float = 0.08,
    max_gap_voxels: float = 4.0,
) -> Mesh:
    """Poisson surface reconstruction.

    Normals are oriented toward the scanner that captured each point (`origins[i]` for points
    with `source_index == i`), which is what makes the surface closed on the correct side.
    Triangles supported by little data (low Poisson density) or far from any scan point are
    removed, so unobserved space is left open rather than filled with invented surface.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud.points)
    if cloud.colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(cloud.colors / 255.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=40))

    normals = np.asarray(pcd.normals).copy()
    if source_index is not None and origins is not None:
        points = np.asarray(pcd.points)
        for i, origin in enumerate(origins):
            mask = source_index == i
            toward = np.asarray(origin) - points[mask]
            flip = np.einsum('ij,ij->i', normals[mask], toward) < 0
            idx = np.where(mask)[0][flip]
            normals[idx] *= -1
    pcd.normals = o3d.utility.Vector3dVector(normals)

    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, trim_density))

    vertices = np.asarray(mesh.vertices)
    far = cKDTree(cloud.points).query(vertices)[0] > voxel * max_gap_voxels
    mesh.remove_vertices_by_mask(far)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()

    colors = None
    if mesh.has_vertex_colors():
        colors = np.clip(np.asarray(mesh.vertex_colors) * 255 + 0.5, 0, 255).astype(np.uint8)
    triangles = np.asarray(mesh.triangles)
    if len(triangles) == 0:
        raise ValueError('Surface reconstruction produced no triangles; the cloud is too sparse.')
    return Mesh(np.asarray(mesh.vertices), triangles, colors)


def write_mesh(path: str, mesh: Mesh) -> None:
    out = o3d.geometry.TriangleMesh()
    out.vertices = o3d.utility.Vector3dVector(mesh.vertices)
    out.triangles = o3d.utility.Vector3iVector(mesh.triangles)
    if mesh.colors is not None:
        out.vertex_colors = o3d.utility.Vector3dVector(mesh.colors / 255.0)
    out.compute_vertex_normals()
    if not o3d.io.write_triangle_mesh(path, out):
        raise IOError(f'Could not write {path}')
