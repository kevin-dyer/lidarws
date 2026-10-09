r"""
Serial protocol driver for the tilt stage Arduino (pure Python).

Firmware protocol (firmware/tilt_stage/tilt_stage.ino), one ASCII
command per line at 115200 8N1, all replies terminated by '\n':

  READY                          - emitted once after reset
  T<angle_deg>                   - reply to 'T' (angle, 4 decimals)
  Z<steps>                       - reply to 'Z'
  M<angle> => T<angle> or LIMIT  - reply to 'M' (out of range)
  C<start>,<stop>                - continuous sweep; replies with sweep
                                   samples "<micros>,<steps>" (~10 Hz)
                                   and a final lone "X" when back at home
  X                              - stop sweep, return home (reply "X")
  A                              - homing; replies "Homing complete" + T line
  S<deg/s>                       - set max speed; reply "Max tilt speed: ..."
  L<0|1>                         - hall sensor state
  R                              - multi-line status dump
  ERR ... / LIMIT / ERROR ...    - error replies

Commands sent while the stage is busy return "ERR BUSY" (except 'X').
"""
import re
import time

DEFAULT_STEPS_PER_DEGREE = 200.0 * 16.0 * 7.6875 / 360.0

# Response matchers, in priority order (checked top to bottom).
DEFAULT_MATCHERS = [
    ('READY', re.compile(r'^READY$')),
    ('ERROR', re.compile(r'^(ERR|ERROR)\b.*')),
    ('LIMIT', re.compile(r'^LIMIT$')),
    ('HOMING_DONE', re.compile(r'^Homing complete$')),
    ('SWEEP_SAMPLE', re.compile(r'^-?\d+,-?\d+$')),
    ('SWEEP_END', re.compile(r'^X$')),
    ('ANGLE', re.compile(r'^T-?\d+\.\d+$')),
    ('STEPS', re.compile(r'^Z-?\d+$')),
]


def match_response(line, matchers=None):
    """Classify a protocol line; returns (name, payload)."""
    for name, pattern in (matchers or DEFAULT_MATCHERS):
        if pattern.match(line):
            if name == 'ANGLE':
                return (name, float(line[1:]))
            if name == 'STEPS':
                return (name, int(line[1:]))
            if name == 'SWEEP_SAMPLE':
                parts = line.split(',')
                return (name, {'micros': int(parts[0]), 'steps': int(parts[1])})
            return (name, line)
    return ('TEXT', line)


class TiltStageError(Exception):
    """Raised on protocol errors, timeouts and closed links."""


class TiltStage:
    """
    Blocking serial driver with a line-buffered response reader.

    Works with any object exposing ``readline()``, ``write()`` and
    ``flush()`` (a serial.Serial or a fake for tests); ``close()`` is
    optional.
    """

    def __init__(self, serial, matchers=None):
        self._serial = serial
        self._matchers = matchers or DEFAULT_MATCHERS
        self._buffer = []
        self.steps_per_degree = None
        self.last_sample_micros = None
        self.last_sample_steps = None
        self.last_sample_angle = None
        self._is_sweeping = False

    @property
    def is_sweeping(self):
        """Whether a sweep is active and its end marker has not been read."""
        return self._is_sweeping

    # -- basic I/O --------------------------------------------------------
    def _classify(self, raw_line):
        if isinstance(raw_line, bytes):
            raw_line = raw_line.decode('ascii', errors='replace')
        line = raw_line.strip()
        if not line:
            return None
        return match_response(line, self._matchers)

    def _drain(self):
        """Consume pending lines (e.g. the startup READY banner)."""
        self._buffer = []
        while True:
            raw = self._serial.readline()
            if not raw:
                break
            resp = self._classify(raw)
            if resp is not None:
                self._buffer.append(resp)
        return list(self._buffer)

    def _next_line(self, timeout=None):
        """Return the next classified line (buffer first, else blocking)."""
        if self._buffer:
            return self._buffer.pop(0)
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            raw = self._serial.readline()
            if not raw:
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError()
                continue
            resp = self._classify(raw)
            if resp is not None:
                return resp
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError()

    def _command(self, text, timeout, expect):
        """Write a command and wait for the first response in ``expect``."""
        deadline = time.monotonic() + timeout
        self._serial.write((text + '\n').encode('ascii'))
        self._serial.flush()
        while True:
            try:
                resp = self._next_line(timeout=min(0.25,
                                                   max(0.01, deadline - time.monotonic())))
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise TiltStageError('timeout waiting for reply to %r' % text)
                continue
            if resp[0] in expect:
                return resp
            if resp[0] == 'ERROR' or resp[0] == 'LIMIT':
                raise TiltStageError(resp[1])

    # -- high level commands ------------------------------------------------
    def connect(self, timeout=5.0):
        """Drain the startup banner and verify the stage answers."""
        self._drain()
        self._command('T', timeout, ('ANGLE',))

    def read_angle(self, timeout=1.0):
        """Query the current tilt angle (degrees)."""
        resp = self._command('T', timeout, ('ANGLE',))
        return resp[1]

    def read_steps(self, timeout=1.0):
        """Query the current step count."""
        resp = self._command('Z', timeout, ('STEPS',))
        return resp[1]

    def move_to(self, angle_deg, timeout=5.0):
        """Move to an absolute angle; returns the angle actually reached."""
        resp = self._command('M%.4f' % angle_deg, timeout, ('ANGLE',))
        return resp[1]

    def next_sample(self, timeout=30.0):
        """
        Wait for the next sweep sample line.

        Returns ``(sample, is_end)`` where ``sample`` is the absolute
        tilt angle in degrees (converted from stepper steps when
        ``steps_per_degree`` is set) and ``is_end`` is True for the
        terminating 'X' line.  Raises ``TimeoutError`` if no sample or
        terminating line arrives within ``timeout`` seconds.
        """
        while True:
            resp = self._next_line(timeout=timeout)
            kind = resp[0]
            if kind == 'SWEEP_SAMPLE':
                self._update_sample(resp[1])
                steps = resp[1]['steps']
                if self.steps_per_degree:
                    return float(steps) / float(self.steps_per_degree), False
                return float(steps), False
            if kind == 'SWEEP_END':
                self._is_sweeping = False
                return None, True
            # any other line (stale echo etc.) - keep waiting

    def start_sweep(self, start_deg, stop_deg, timeout=3.0):
        """
        Issue 'C start,stop' and wait for the first sweep sample.

        A protocol error (busy / limits) is raised as TiltStageError.
        """
        cmd = 'C%.4f,%.4f' % (start_deg, stop_deg)
        deadline = time.monotonic() + timeout
        self._serial.write((cmd + '\n').encode('ascii'))
        self._serial.flush()
        self._is_sweeping = True
        while True:
            try:
                resp = self._next_line(timeout=min(0.25,
                                                   max(0.01, deadline - time.monotonic())))
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise TiltStageError('timeout waiting for sweep to start')
                continue
            if resp[0] == 'SWEEP_SAMPLE':
                self._update_sample(resp[1])
                return self.last_sample_angle
            if resp[0] == 'ERROR' or resp[0] == 'LIMIT':
                self._is_sweeping = False
                raise TiltStageError(resp[1])

    def stop_sweep(self, timeout=30.0):
        """Send 'X' and wait until the stage is home and idle again."""
        self._serial.write(b'X\n')
        self._serial.flush()
        return self.wait_sweep_end(timeout=timeout)

    def wait_sweep_end(self, timeout=60.0):
        """Consume sweep samples until the terminating 'X' arrives."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                resp = self._next_line(timeout=min(0.5,
                                                   max(0.01, deadline - time.monotonic())))
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise TiltStageError('timeout waiting for sweep to end')
                continue
            if resp[0] == 'SWEEP_SAMPLE':
                self._update_sample(resp[1])
                continue
            if resp[0] == 'SWEEP_END':
                self._is_sweeping = False
                return
            if resp[0] == 'ERROR':
                raise TiltStageError(resp[1])

    def home(self, timeout=60.0):
        """Run the homing sequence 'A'; returns the angle after homing."""
        self._command('A', 1.0, ('TEXT', 'HOMING_DONE'))
        self._wait_named('HOMING_DONE', timeout)
        return self.read_angle(timeout=2.0)

    def _wait_named(self, name, timeout):
        deadline = time.monotonic() + timeout
        while True:
            try:
                resp = self._next_line(timeout=min(0.25,
                                                   max(0.01, deadline - time.monotonic())))
            except TimeoutError:
                if time.monotonic() >= deadline:
                    raise TiltStageError('timeout waiting for %r' % name)
                continue
            if resp[0] == name:
                return resp
            if resp[0] == 'ERROR':
                raise TiltStageError(resp[1])

    def set_speed(self, deg_per_sec, timeout=1.0):
        self._command('S%.2f' % deg_per_sec, timeout, ('TEXT',))

    def set_steps_per_degree(self, value):
        self.steps_per_degree = float(value)

    def _update_sample(self, sample):
        self.last_sample_micros = sample['micros']
        self.last_sample_steps = sample['steps']
        if self.steps_per_degree:
            self.last_sample_angle = sample['steps'] / self.steps_per_degree

    # -- lifecycle ----------------------------------------------------------
    def close(self):
        closer = getattr(self._serial, 'close', None)
        if closer:
            closer()
