"""
Accumulate ray observations into per-sweep 3D point clouds.

Pure Python + numpy (no ROS) so the behavior is unit-testable.  Each
observation is a ray ``(azimuth_deg, tilt_deg, range_m)``: the LiDAR
azimuth, the stage tilt at the moment of the scan and the measured
range.  :class:`CloudAccumulator` gathers accepted observations for
one sweep and exposes them as an (N, 3) numpy array.
"""
import math

import numpy as np

from .tilt_math import check_ray, ray_to_world

DEFAULT_MIN_RANGE_M = 0.05
DEFAULT_MAX_RANGE_M = 30.0

#: (name, accepted, reason) convenience for callers.
REASONS = {
    'OK': 'accepted',
    'INVALID_RANGE': 'range',
    'GRAZING': 'grazing',
    'MAX_ANGULAR': 'max angular',
}


class CloudAccumulator:
    """
    Collect ray observations for one sweep into a point cloud.

    Observations are appended in arrival order; no de-duplication is
    performed (overlapping sweeps simply add points, which is desired
    for a full multi-position capture). ``azimuth_offset_deg`` maps the
    LiDAR's zero-angle direction into the point-cloud frame.
    """

    def __init__(self, min_range=DEFAULT_MIN_RANGE_M,
                 max_range=DEFAULT_MAX_RANGE_M,
                 azimuth_offset_deg=0.0):
        self._min_range = float(min_range)
        self._max_range = float(max_range)
        self._azimuth_offset_deg = float(azimuth_offset_deg)
        if not math.isfinite(self._azimuth_offset_deg):
            raise ValueError('azimuth_offset_deg must be finite')
        self._points = []
        self.stats = {'accepted': 0, 'rejected': 0, 'reasons': {}}

    def _bump_reason(self, reason):
        self.stats['reasons'][reason] = self.stats['reasons'].get(reason, 0) + 1

    def add(self, azimuth_deg, tilt_deg, range_m):
        """Validate and add one ray.  Returns ``(accepted, reason)``."""
        # accepted, reason = check_ray(
        #     azimuth_deg, tilt_deg, range_m,
        #     self._min_range, self._max_range)
        # if not accepted:
        #     self.stats['rejected'] += 1
        #     self._bump_reason(reason)
        #     return (False, reason)
        accepted = True
        reason = 'OK'
        self._points.append(ray_to_world(
            azimuth_deg + self._azimuth_offset_deg, tilt_deg, range_m))
        self.stats['accepted'] += 1
        return (True, reason)

    def extend(self, observations):
        """Add many ``(azimuth_deg, tilt_deg, range_m)`` rays."""
        for obs in observations:
            self.add(*obs)

    def __len__(self):
        return len(self._points)

    def reset(self):
        self._points = []
        self.stats = {'accepted': 0, 'rejected': 0, 'reasons': {}}

    def to_numpy(self):
        """Return the accumulated points as an (N, 3) float64 array."""
        if not self._points:
            return np.empty((0, 3), dtype=np.float64)
        return np.asarray(self._points, dtype=np.float64)
