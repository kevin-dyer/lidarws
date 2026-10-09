"""Tests for the tilt-stage serial protocol driver."""
from tilt_coordinator.tilt_serial import TiltStage


class FakeSerial:
    def __init__(self, lines):
        self.lines = list(lines)
        self.writes = []

    def readline(self):
        return self.lines.pop(0) if self.lines else b''

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass


def test_sweep_state_and_samples():
    serial = FakeSerial([
        b'1000,320\n',
        b'2000,640\n',
        b'3000,960\n',
        b'X\n',
    ])
    stage = TiltStage(serial)
    stage.set_steps_per_degree(32)

    assert not stage.is_sweeping
    stage.start_sweep(-10.0, 10.0)
    assert stage.is_sweeping
    assert stage.last_sample_micros == 1000
    assert stage.last_sample_steps == 320

    sample, is_end = stage.next_sample()
    assert sample == 20.0
    assert not is_end

    stage.stop_sweep()
    assert not stage.is_sweeping
    assert serial.writes == [b'C-10.0000,10.0000\n', b'X\n']


def test_read_timeout_does_not_mean_serial_link_closed():
    serial = FakeSerial([b'', b'T0.0000\n'])
    stage = TiltStage(serial)

    assert stage.read_angle() == 0.0


def test_set_speed_sends_sweep_speed_command():
    serial = FakeSerial([b'Max tilt speed: 2.0 deg/s\n'])
    stage = TiltStage(serial)

    stage.set_speed(2.0)

    assert serial.writes == [b'S2.00\n']


def test_move_to_zero_uses_absolute_move_command():
    serial = FakeSerial([b'T0.0000\n'])
    stage = TiltStage(serial)

    assert stage.move_to(0.0, timeout=60.0) == 0.0
    assert serial.writes == [b'M0.0000\n']
