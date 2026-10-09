"""
Pure geometry for the tilt-stage 3D capture (no ROS, unit-testable).

Coordinate model
----------------
* LiDAR local frame: the beam lies in a horizontal plane; a ray with
  azimuth ``a`` (degrees, 0 = +X, CCW about +Y) has direction
  ``(cos a, sin a, 0)``.
* Tilt ``t`` (degrees, positive = nose up): the LiDAR frame is rotated
  about the pivot axis (world +Y) so that local +X tilts toward +Z.
  The world direction of a ray is :func:`rotate_about_y` applied to
  its local direction.
* The stage axis (local +X, world) is :func:`stage_axis`.

``check_ray`` applies the same beam-quality filters as the firmware
plan: range window, near-vertical (grazing) rejection and a max
angular deviation from the stage axis.
"""
import math


def wrap_angle(deg):
    """
    Wrap an angle in degrees to [-180, 180).

    >>> wrap_angle(370.0)
    10.0
    >>> wrap_angle(-200.0)
    160.0
    >>> wrap_angle(180.0)
    -180.0
    """
    return ((deg + 180.0) % 360.0) - 180.0


def wrap_full(deg):
    """
    Wrap an angle in degrees to [0, 360).

    >>> wrap_full(370.0)
    10.0
    >>> wrap_full(-10.0)
    350.0
    """
    return deg % 360.0


def ray_direction(azimuth_deg):
    """
    Return the unit beam direction in the LiDAR local (scan) plane.

    >>> ray_direction(0.0)
    (1.0, 0.0, 0.0)
    >>> [round(v, 6) for v in ray_direction(90.0)]
    [0.0, 1.0, 0.0]
    """
    a = math.radians(azimuth_deg)
    return (math.cos(a), math.sin(a), 0.0)


def rotate_about_y(point, tilt_deg):
    """
    Rotate a LiDAR-local point/direction into the world frame.

    Positive ``tilt_deg`` tilts the local +X axis up toward +Z
    (the scan plane pitches up about the pivot axis Y).

    >>> rotate_about_y((1.0, 0.0, 0.0), 0.0)
    (1.0, 0.0, 0.0)
    >>> [round(v, 9) for v in rotate_about_y((1.0, 0.0, 0.0), 90.0)]
    [0.0, 0.0, 1.0]
    >>> [round(v, 6) for v in rotate_about_y((1.0, 0.0, 0.0), 30.0)]
    [0.866025, 0.0, 0.5]
    >>> rotate_about_y((0.0, 1.0, 0.0), 45.0)   # pivot axis is invariant
    (0.0, 1.0, 0.0)
    """
    t = math.radians(tilt_deg)
    c, s = math.cos(t), math.sin(t)
    x, y, z = point
    return (c * x - s * z, y, s * x + c * z)


def stage_axis(tilt_deg):
    """
    Return the unit direction of the stage axis (LiDAR local +X) in the world.

    >>> stage_axis(0.0)
    (1.0, 0.0, 0.0)
    >>> [round(v, 9) for v in stage_axis(90.0)]
    [0.0, 0.0, 1.0]
    """
    t = math.radians(tilt_deg)
    return (math.cos(t), 0.0, math.sin(t))


def angle_between(v1, v2):
    """
    Return the angle in degrees between two 3D vectors (0..180).

    >>> angle_between((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    90.0
    >>> angle_between((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    180.0
    """
    dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def ray_to_world(azimuth_deg, tilt_deg, range_m):
    """
    Convert a ray (LiDAR azimuth, stage tilt, range) to a world point.

    >>> ray_to_world(0.0, 0.0, 2.0)
    (2.0, 0.0, 0.0)
    >>> [round(v, 9) for v in ray_to_world(90.0, 0.0, 1.0)]
    [0.0, 1.0, 0.0]
    >>> [round(v, 9) for v in ray_to_world(0.0, 90.0, 1.0)]
    [0.0, 0.0, 1.0]
    >>> [round(v, 6) for v in ray_to_world(90.0, 90.0, 1.0)]
    [0.0, 1.0, 0.0]
    >>> [round(v, 6) for v in ray_to_world(45.0, 90.0, 1.0)]
    [0.0, 0.707107, 0.707107]
    """
    local = (
        range_m * math.cos(math.radians(azimuth_deg)),
        range_m * math.sin(math.radians(azimuth_deg)),
        0.0,
    )
    x, y, z = rotate_about_y(local, tilt_deg)
    return (x, y, z)


# Ray quality reasons
OK = 'OK'
INVALID_RANGE = 'INVALID_RANGE'
GRAZING = 'GRAZING'
MAX_ANGULAR = 'MAX_ANGULAR'

#: Beams closer than this to the scan-plane normal (|z| > 0.99) are
#: treated as grazing/vertical noise.
GRAZING_Z_THRESHOLD = 0.99


def max_angular_limit(tilt_deg):
    """
    Return the max allowed angle between the beam and the stage axis.

    ``90 - asin(|sin tilt|)``: at tilt 0 the beam may deviate up to 90
    degrees; as the stage approaches vertical the beam must stay
    aligned with the (vertical) axis.

    >>> round(max_angular_limit(0.0), 6)
    90.0
    >>> round(max_angular_limit(30.0), 6)
    60.0
    >>> round(max_angular_limit(90.0), 6)
    0.0
    """
    return 90.0 - math.degrees(math.asin(abs(math.sin(math.radians(tilt_deg)))))


def check_ray(azimuth_deg, tilt_deg, range_m, min_range=0.1, max_range=12.0):
    """
    Classify one ray observation and return ``(accepted, reason)``.

    Order: range window, grazing, max angular deviation from the stage
    axis.  ``azimuth_deg`` is the LiDAR-local azimuth (0..360).

    >>> check_ray(0.0, 0.0, 0.05)
    (False, 'INVALID_RANGE')
    >>> check_ray(0.0, 0.0, 5.0)
    (True, 'OK')
    >>> check_ray(0.0, 30.0, 2.0)
    (True, 'OK')
    >>> check_ray(90.0, 90.0, 2.0)   # beam along pivot axis, max_ang = 0
    (False, 'MAX_ANGULAR')
    """
    if (not math.isfinite(range_m) or range_m < min_range or range_m > max_range):
        return (False, INVALID_RANGE)
    world_beam = rotate_about_y(ray_direction(azimuth_deg), tilt_deg)
    if abs(world_beam[2]) > GRAZING_Z_THRESHOLD:
        return (False, GRAZING)
    limit = max_angular_limit(tilt_deg)
    if angle_between(world_beam, stage_axis(tilt_deg)) > limit:
        return (False, MAX_ANGULAR)
    return (True, OK)
