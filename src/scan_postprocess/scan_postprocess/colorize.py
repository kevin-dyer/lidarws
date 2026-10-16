"""Assign RGB colours from a reference photo to a scanner-frame point cloud.

The photo is assumed to be taken from the scanner position, looking along the scanner's
forward (+x) axis. Only points that can be matched to the photo with high confidence get a
colour; see `colorize_points` for the criteria.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageOps
from scipy import ndimage

# Scanner axes -> camera axes. The raw PLY frame has z pointing down (the mobile app flips z only
# for display), so the default is x forward / y right / z down.
FRAMES = {
    'frd': np.array([1.0, 1.0, 1.0]),  # image right = +y, image down = +z
    'flu': np.array([1.0, -1.0, -1.0]),  # image right = -y, image down = -z
}


@dataclass
class ColorizeResult:
    colors: np.ndarray  # (N, 3) uint8
    confident: np.ndarray  # (N,) bool
    stats: dict


def load_image(path: str) -> np.ndarray:
    image = ImageOps.exif_transpose(Image.open(path)).convert('RGB')
    return np.asarray(image)


def colorize_points(
    points: np.ndarray,
    image: np.ndarray,
    hfov_deg: float = 65.0,
    frame: str = 'frd',
    min_range: float = 0.3,
    max_range: float = 30.0,
    edge_margin: float = 0.03,
    occlusion_cell_px: int = 4,
    occlusion_ratio: float = 1.08,
    occlusion_slack: float = 0.08,
    max_color_stddev: float = 22.0,
    fill_color: tuple = (128, 128, 128),
) -> ColorizeResult:
    """Colours the points that are visible in `image` with high confidence.

    A point is coloured only if all of these hold:
      * it lies in front of the camera within [min_range, max_range] metres;
      * it projects inside the image, away from the borders where lens distortion is largest;
      * it is not hidden behind a nearer point along the same viewing ray (z-buffer test);
      * the photo is locally uniform at that pixel (no strong edge), so a small calibration or
        registration error cannot pick the colour of the wrong surface.
    """
    if frame not in FRAMES:
        raise ValueError(f'Unknown frame {frame!r}; use one of {sorted(FRAMES)}.')
    height, width = image.shape[:2]
    focal = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    sign = FRAMES[frame]

    depth = points[:, 0]
    in_front = depth > 1e-6
    safe_depth = np.where(in_front, depth, 1.0)
    u = width / 2.0 + focal * sign[1] * points[:, 1] / safe_depth
    v = height / 2.0 + focal * sign[2] * points[:, 2] / safe_depth
    distance = np.linalg.norm(points, axis=1)

    mx, my = edge_margin * width, edge_margin * height
    visible = (
        in_front
        & (distance >= min_range)
        & (distance <= max_range)
        & (u >= mx) & (u <= width - 1 - mx)
        & (v >= my) & (v <= height - 1 - my)
    )

    ui = np.clip(np.round(u).astype(int), 0, width - 1)
    vi = np.clip(np.round(v).astype(int), 0, height - 1)

    # Occlusion: keep points no farther than the nearest surface seen in their pixel cell.
    cells_x = width // occlusion_cell_px + 1
    cells_y = height // occlusion_cell_px + 1
    cell = (vi // occlusion_cell_px) * cells_x + (ui // occlusion_cell_px)
    nearest = np.full(cells_x * cells_y, np.inf)
    np.minimum.at(nearest, cell[visible], depth[visible])
    unoccluded = depth <= nearest[cell] * occlusion_ratio + occlusion_slack

    img = image.astype(np.float32)
    mean = ndimage.uniform_filter(img, size=(3, 3, 1))
    local_mean = ndimage.uniform_filter(img, size=(7, 7, 1))
    local_sq = ndimage.uniform_filter(img * img, size=(7, 7, 1))
    stddev = np.sqrt(np.maximum(local_sq - local_mean ** 2, 0.0)).mean(axis=2)
    uniform = stddev[vi, ui] <= max_color_stddev

    confident = visible & unoccluded & uniform
    colors = np.tile(np.array(fill_color, dtype=np.uint8), (len(points), 1))
    colors[confident] = np.clip(mean[vi[confident], ui[confident]] + 0.5, 0, 255).astype(np.uint8)
    stats = {
        'points': int(len(points)),
        'in_view': int(visible.sum()),
        'occluded': int((visible & ~unoccluded).sum()),
        'ambiguous_edges': int((visible & unoccluded & ~uniform).sum()),
        'colored': int(confident.sum()),
    }
    return ColorizeResult(colors, confident, stats)
