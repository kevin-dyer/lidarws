"""
Scan/sweep coordination state machine (pure logic, no ROS).

For each capture position the node makes sure the stage is idle,
homes if needed, moves to the target angle, then starts the sweep and
accepts LiDAR frames while sweep samples are consumed.  The LiDAR
frame source, the serial driver and the clock are injected, so this
class is fully unit-testable.
"""
import math
import time

from .cloud_accumulator import CloudAccumulator

MAX_TILT_ANGLE_DEG = 45.0
SWEEP_SAMPLE_RATE_HZ = 100.0


def estimate_max_sweep_samples(start_angle, stop_angle, speed_deg_s):
    """Estimate a generous timeout guard for a 100 Hz tilt sweep."""
    if not math.isfinite(speed_deg_s) or speed_deg_s <= 0.0:
        raise ValueError('sweep speed must be finite and greater than zero')
    sweep_duration = abs(float(stop_angle) - float(start_angle)) / speed_deg_s
    return int(math.ceil(
        sweep_duration * SWEEP_SAMPLE_RATE_HZ * 1.5
        + 5.0 * SWEEP_SAMPLE_RATE_HZ))


class State:
    """Named sweep position with per-position parameters."""

    def __init__(self, name, target_angle, sweep_start, sweep_stop,
                 max_samples=1000):
        self.name = name
        self.target_angle = float(target_angle)
        self.sweep_start = float(sweep_start)
        self.sweep_stop = float(sweep_stop)
        self.max_samples = int(max_samples)


def validate_tilt_states(states, max_angle=MAX_TILT_ANGLE_DEG):
    """Reject capture settings outside the tilt stage's firmware limits."""
    for state in states:
        for name in ('target_angle', 'sweep_start', 'sweep_stop'):
            angle = getattr(state, name)
            if not math.isfinite(angle) or abs(angle) > max_angle:
                raise ValueError(
                    '%s %s=%.3f exceeds the tilt stage limit of +/-%.1f degrees'
                    % (state.name, name, angle, max_angle))


class SweepCoordinator:
    """
    Run one capture position and produce a point cloud.

    Parameters
    ----------
    stage:
        Object implementing the :class:`TiltStage` interface
        (``is_sweeping``, ``read_angle``, ``move_to``, ``start_sweep``,
        ``next_sample``, ``stop_sweep``, ``home``).
    frame_source: callable
        ``frame_source() -> (timestamp, [(azimuth_deg, range_m), ...])``
        for the latest LiDAR frame; must raise when no frame is ready.
    cloud: CloudAccumulator, optional
        Reused accumulator (points persist across positions).
    sample_timeout: float
        How long to wait for each sweep sample.
    clock: callable
        Monotonic clock (defaults to ``time.monotonic``).

    """

    def __init__(self, stage, frame_source, cloud=None,
                 sample_timeout=30.0, clock=time.monotonic,
                 on_cloud_update=None):
        self.stage = stage
        self.frame_source = frame_source
        self.cloud = cloud if cloud is not None else CloudAccumulator()
        self.sample_timeout = float(sample_timeout)
        self.clock = clock
        self.on_cloud_update = on_cloud_update
        self.log = []
        self._current_tilt = 0.0

    # -- diagnostics ------------------------------------------------------
    def _log(self, message):
        self.log.append(message)

    def _now(self):
        return self.clock()

    # -- frame intake -------------------------------------------------------
    def _collect_frames(self):
        """Drain all complete LiDAR frames currently ready."""
        while True:
            try:
                ts, rays = self.frame_source()
            except Exception:  # no frame ready yet
                return
            self._ingest_frame(ts, rays)

    def _ingest_frame(self, timestamp, rays):
        tilt = self._current_tilt
        accepted = rejected = 0
        for azimuth_deg, range_m in rays:
            ok, _reason = self.cloud.add(azimuth_deg, tilt, range_m)
            if ok:
                accepted += 1
            else:
                rejected += 1
        self._log('frame t=%.3f tilt=%.1f rays=%d acc=%d rej=%d' % (
            timestamp, tilt, len(rays), accepted, rejected))
        if accepted and self.on_cloud_update is not None:
            self.on_cloud_update()

    # -- position run -----------------------------------------------------
    def run_position(self, state):
        """
        Execute one capture position; returns the stage angle used.

        Sequence per the plan:
          1. If a sweep is active, send 'X' and wait for it to end.
          2. Home (only if the stage is not already at 0).
          3. 'M target' (skipped if already at the target angle).
          4. 'C start,stop' - start the sweep.
          5. Consume sweep samples, feeding ready LiDAR frames into the
             accumulator, until the sweep terminates.
        """
        stage = self.stage

        self._log('position %s: target=%.1f sweep=[%.1f..%.1f]' % (
            state.name, state.target_angle, state.sweep_start, state.sweep_stop))

        # 1. make sure we are idle
        if stage.is_sweeping:
            self._log('position %s: active sweep, stopping' % state.name)
            stage.stop_sweep(timeout=self.sample_timeout)

        # 2. homing - only if not at home already
        current = stage.read_angle()
        if abs(current) > 1e-6:
            self._log('position %s: homing' % state.name)
            stage.home(timeout=self.sample_timeout)

        # 3. move to target
        current = stage.read_angle()
        if abs(current - state.target_angle) > 1e-3:
            self._log('position %s: M %.3f' % (state.name, state.target_angle))
            reached = stage.move_to(
                state.target_angle, timeout=self.sample_timeout)
        else:
            reached = current
            self._log('position %s: already at %.3f' % (state.name, reached))

        # 4. start sweep; the first sample has already been consumed
        self._current_tilt = reached
        self._log('position %s: C %.3f,%.3f' % (
            state.name, state.sweep_start, state.sweep_stop))
        first_sample = stage.start_sweep(
            state.sweep_start, state.sweep_stop,
            timeout=self.sample_timeout)
        if first_sample is not None:
            self._current_tilt = first_sample
        self._collect_frames()

        # 5. consume sweep until the terminating 'X'
        self._wait_sweep_end(state)
        self._collect_frames()
        return reached

    def _wait_sweep_end(self, state):
        """Consume sweep samples until the terminating 'X' arrives."""
        stage = self.stage
        samples = 0
        while True:
            self._collect_frames()
            if not stage.is_sweeping:
                self._log('position %s: sweep ended after %d samples' % (
                    state.name, samples))
                return
            try:
                _sample, is_end = stage.next_sample(timeout=self.sample_timeout)
            except TimeoutError:
                if samples >= state.max_samples:
                    self._log('position %s: sample timeout, stopping sweep'
                              % state.name)
                    stage.stop_sweep(timeout=self.sample_timeout)
                    return
                continue
            if is_end:
                self._log('position %s: sweep ended after %d samples' % (
                    state.name, samples))
                return
            if _sample is not None:
                self._current_tilt = _sample
            samples += 1
            if samples >= state.max_samples:
                self._log('position %s: max samples, stopping sweep'
                          % state.name)
                stage.stop_sweep(timeout=self.sample_timeout)
                return
