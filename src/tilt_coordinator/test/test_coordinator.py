"""Logic tests for tilt_coordinator using a fake tilt stage (no ROS)."""
import doctest

import numpy as np
import pytest

from tilt_coordinator.cloud_accumulator import CloudAccumulator
from tilt_coordinator.coordinator import (
    SweepCoordinator,
    State,
    estimate_max_sweep_samples,
    validate_tilt_states,
)
from tilt_coordinator.tilt_math import (
    check_ray,
    max_angular_limit,
    ray_to_world,
)


class FakeStage:
    """Mimics the TiltStage interface with a scripted sweep."""

    def __init__(self, sweep_length=5, start_angle=45.0, sweeping=False):
        self.angle = start_angle
        self.is_sweeping = sweeping
        self.sweep = 0.0
        self.sweep_length = sweep_length
        self.sweep_samples_left = sweep_length
        self.commands = []
        self.start_sweep_timeouts = []

    def read_angle(self):
        return self.angle

    def home(self, timeout=None):
        self.commands.append('H')
        self.angle = 0.0
        return 0.0

    def move_to(self, target, timeout=None):
        self.commands.append('M %.3f' % target)
        self.angle = target
        return target

    def start_sweep(self, a, b, timeout=None):
        self.commands.append('C %.3f,%.3f' % (a, b))
        self.start_sweep_timeouts.append(timeout)
        self.is_sweeping = True
        self.sweep = a
        self.sweep_samples_left = self.sweep_length
        return a

    def next_sample(self, timeout=None):
        if not self.is_sweeping:
            raise TimeoutError('not sweeping')
        if self.sweep_samples_left == 0:
            self.is_sweeping = False
            return (self.sweep, True)
        self.sweep_samples_left -= 1
        self.sweep += 1.0
        return (self.sweep, False)

    def stop_sweep(self, timeout=None):
        self.commands.append('X')
        self.next_sample()
        return self.sweep

    def close(self):
        pass


class FrameSource:
    """Returns scripted frames, then raises like an empty queue."""

    def __init__(self, frames):
        self.frames = list(frames)

    def __call__(self):
        if not self.frames:
            raise TimeoutError('no frame ready')
        return self.frames.pop(0)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 0.01
        return self.t


def frame(rays):
    return (1.5, rays)


# ---------------------------------------------------------------------------
# math
def test_tilt_math_doctests():
    import tilt_coordinator.tilt_math as tm

    results = doctest.testmod(tm, verbose=False)
    assert results.failed == 0


def test_max_angular_limit():
    assert max_angular_limit(0.0) == pytest.approx(90.0)
    assert max_angular_limit(30.0) == pytest.approx(60.0)
    assert max_angular_limit(90.0) == pytest.approx(0.0)


def test_sweep_sample_limit_covers_full_sweep_at_configured_speed():
    assert estimate_max_sweep_samples(-45.0, 45.0, 2.0) == 7250
    with pytest.raises(ValueError, match='speed must be finite and greater than zero'):
        estimate_max_sweep_samples(-45.0, 45.0, 0.0)


def test_ray_to_world():
    assert np.allclose(ray_to_world(0.0, 0.0, 2.0), (2.0, 0.0, 0.0))
    assert np.allclose(ray_to_world(90.0, 0.0, 2.0), (0.0, 2.0, 0.0))
    assert np.allclose(ray_to_world(0.0, 90.0, 2.0), (0.0, 0.0, 2.0))
    expected = np.array([2 * 0.8660254, 0.0, 2 * 0.5])
    assert np.allclose(ray_to_world(0.0, 30.0, 2.0), expected)


def test_cloud_accumulator_applies_lidar_mount_azimuth_offset():
    acc = CloudAccumulator(azimuth_offset_deg=90.0)

    assert acc.add(0.0, 0.0, 1.0) == (True, 'OK')
    assert np.allclose(acc.to_numpy()[0], [0.0, 1.0, 0.0], atol=1e-12)


def test_check_ray_rejections():
    assert check_ray(0.0, 0.0, 0.01) == (False, 'INVALID_RANGE')
    assert check_ray(0.0, 0.0, 99.0) == (False, 'INVALID_RANGE')
    assert check_ray(0.0, 30.0, 1.0) == (True, 'OK')
    assert check_ray(90.0, 90.0, 1.0) == (False, 'MAX_ANGULAR')


def test_validate_tilt_states_enforces_firmware_limits():
    validate_tilt_states([
        State('A', 0.0, 0.0, 45.0),
        State('B', 45.0, 0.0, -45.0),
    ])

    with pytest.raises(ValueError, match=r'A sweep_stop=90.000.*\+/-45.0'):
        validate_tilt_states([State('A', 0.0, 0.0, 90.0)])

    with pytest.raises(ValueError, match='B target_angle'):
        validate_tilt_states([State('B', float('nan'), 0.0, 0.0)])


# ---------------------------------------------------------------------------
# accumulator
def test_cloud_accumulator():
    acc = CloudAccumulator(min_range=0.1, max_range=12.0)
    assert acc.add(0.0, 0.0, 1.0) == (True, 'OK')
    assert acc.add(0.0, 0.0, 0.01) == (False, 'INVALID_RANGE')
    pts = acc.to_numpy()
    assert pts.shape == (1, 3)
    assert np.allclose(pts[0], [1.0, 0.0, 0.0])
    assert acc.stats == {'accepted': 1, 'rejected': 1,
                         'reasons': {'INVALID_RANGE': 1}}


def test_default_cloud_range_accepts_longer_lidar_returns():
    acc = CloudAccumulator()

    assert acc.add(0.0, 0.0, 20.0) == (True, 'OK')
    assert acc.add(0.0, 0.0, 0.02) == (False, 'INVALID_RANGE')
    assert len(acc) == 1


# ---------------------------------------------------------------------------
# coordinator
def test_run_position_protocol():
    stage = FakeStage(start_angle=45.0)
    src = FrameSource([
        frame([(0.0, 1.0), (30.0, 1.0)]),
        frame([(60.0, 1.0), (90.0, 1.0)]),
    ])
    coord = SweepCoordinator(stage, src, clock=FakeClock())
    reached = coord.run_position(State('A', 0.0, 0.0, 90.0))

    assert reached == 0.0
    assert len(coord.cloud) == 4
    # homed (was at 45 deg), already at target 0 (no M), sweep started
    assert stage.commands == ['H', 'C 0.000,90.000']
    assert not stage.is_sweeping


def test_run_position_moves_when_needed():
    stage = FakeStage(start_angle=45.0)
    src = FrameSource([frame([(0.0, 1.0)])])
    coord = SweepCoordinator(stage, src, clock=FakeClock())
    coord.run_position(State('A', 30.0, 0.0, 60.0))

    assert stage.commands == ['H', 'M 30.000', 'C 0.000,60.000']
    assert len(coord.cloud) == 1


def test_sweep_start_uses_sample_timeout_after_travel_to_start():
    stage = FakeStage(start_angle=45.0)
    coord = SweepCoordinator(
        stage,
        FrameSource([]),
        sample_timeout=30.0,
        clock=FakeClock(),
    )

    coord.run_position(State('B', 45.0, 0.0, -45.0))

    assert stage.start_sweep_timeouts == [30.0]


def test_frames_are_collected_and_cloud_updates_during_sweep():
    stage = FakeStage(start_angle=0.0)
    updates = []
    coord = SweepCoordinator(
        stage,
        FrameSource([frame([(0.0, 1.0)])]),
        on_cloud_update=lambda: updates.append(len(coord.cloud)),
    )

    coord.run_position(State('A', 0.0, -15.0, 15.0))

    assert updates == [1]
    assert coord.cloud.to_numpy()[0, 2] < 0.0


def test_active_sweep_is_stopped_first():
    stage = FakeStage(start_angle=0.0, sweeping=True, sweep_length=3)
    src = FrameSource([])
    coord = SweepCoordinator(stage, src, clock=FakeClock())
    coord.run_position(State('A', 0.0, 0.0, 90.0))

    # first command must be 'X' (stop the in-progress sweep)
    assert stage.commands[0] == 'X'
    assert 'C 0.000,90.000' in stage.commands
    assert not stage.is_sweeping


def test_no_frames_produced_when_source_empty():
    stage = FakeStage(start_angle=0.0)
    coord = SweepCoordinator(stage, FrameSource([]), clock=FakeClock())
    coord.run_position(State('A', 0.0, 0.0, 90.0))
    assert len(coord.cloud) == 0
    assert not stage.is_sweeping
