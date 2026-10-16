"""Colour a merged, registered point cloud from every calibrated photo and blend the results.

Each photo is projected onto the whole merged cloud (so it can colour surfaces that another scan
captured), occlusion is tested against the merged surface, and where several photos see a point
the colours are combined with weights favouring head-on, close views. Photos that disagree with
the consensus are dropped for that point.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .colorize import colorize_points

OUTLIER = 45.0  # RGB distance from the consensus beyond which a view is dropped (needs 3+ photos)


@dataclass
class BlendResult:
    colors: np.ndarray  # (N, 3) uint8
    confident: np.ndarray  # (N,) bool
    views: np.ndarray  # (N,) photos that coloured each point
    disagreement: float  # mean RGB distance between views of the same point before blending
    stats: dict


def estimate_normals(points: np.ndarray, viewpoints: np.ndarray) -> np.ndarray:
    import open3d as o3d
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    normals = np.asarray(pcd.normals).copy()
    flip = np.einsum('ij,ij->i', normals, viewpoints - points) < 0
    normals[flip] *= -1
    return normals


def blend_colors(points, source_index, images, cameras, transforms, frame='frd', **kwargs) -> BlendResult:
    """`points` are in the registered frame; `source_index[k]` is the scan each point came from."""
    n = len(points)
    origins = np.array([np.asarray(t)[:3, 3] for t in transforms])
    normals = estimate_normals(points, origins[np.asarray(source_index)])
    cols, weights = [], []
    for i, (image, camera, t) in enumerate(zip(images, cameras, transforms)):
        if image is None or camera is None:
            continue
        t = np.asarray(t)
        inv = np.linalg.inv(t)
        local = points @ inv[:3, :3].T + inv[:3, 3]
        res = colorize_points(local, image, camera=camera, frame=frame, **kwargs)
        to_cam = origins[i] + t[:3, :3] @ camera.translation - points
        dist = np.linalg.norm(to_cam, axis=1)
        cos = np.abs(np.einsum('ij,ij->i', normals, to_cam / np.maximum(dist, 1e-6)[:, None]))
        w = np.where(res.confident & (cos > 0.2), cos ** 2 / np.maximum(dist, 0.5), 0.0)
        cols.append(res.colors.astype(np.float32))
        weights.append(w.astype(np.float32))
    if not cols:
        return BlendResult(np.full((n, 3), 128, np.uint8), np.zeros(n, bool), np.zeros(n, int), 0.0, {})
    C, W = np.stack(cols), np.stack(weights)  # (K, N, 3), (K, N)
    seen = W > 0
    views = seen.sum(axis=0)

    def mean(w):
        return (C * w[..., None]).sum(axis=0) / np.maximum(w.sum(axis=0), 1e-9)[:, None]

    consensus = mean(W)
    dist = np.linalg.norm(C - consensus[None], axis=2)
    multi = views >= 2
    disagreement = float(dist[:, multi][seen[:, multi]].mean()) if multi.any() else 0.0
    if len(cols) >= 3:
        keep = seen & (dist < OUTLIER)
        best = np.argmax(W, axis=0)
        cols_idx = np.arange(n)
        keep[best, cols_idx] |= seen[best, cols_idx]
        W = np.where(keep, W, 0.0)
        consensus = mean(W)
    confident = W.sum(axis=0) > 0
    colors = np.full((n, 3), 128, np.uint8)
    colors[confident] = np.clip(consensus[confident] + 0.5, 0, 255).astype(np.uint8)
    stats = {'points': n, 'colored': int(confident.sum()), 'multi_view': int(multi.sum())}
    return BlendResult(colors, confident, views, disagreement, stats)
