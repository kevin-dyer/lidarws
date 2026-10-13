"""Tests for the USB mux protocol and its PTY endpoints."""
import os

import pytest

from tilt_coordinator.usb_serial_bridge import (
    CHANNEL_CONTROL,
    CHANNEL_LIDAR,
    CHANNEL_TILT,
    FrameDecoder,
    PtySerial,
    encode_frame,
)


def test_frame_round_trip_across_partial_reads():
    decoder = FrameDecoder()
    packet = encode_frame(CHANNEL_LIDAR, b'\xa5\x5a\x20\x00')

    assert decoder.feed(packet[:4]) == []
    assert decoder.feed(packet[4:]) == [
        (CHANNEL_LIDAR, b'\xa5\x5a\x20\x00')]


def test_decoder_skips_noise_and_rejects_bad_crc():
    decoder = FrameDecoder()
    valid = encode_frame(CHANNEL_CONTROL, b'PONG')
    invalid = bytearray(encode_frame(CHANNEL_TILT, b'T0.0\n'))
    invalid[-1] ^= 0xff

    assert decoder.feed(b'noise' + invalid + valid) == [
        (CHANNEL_CONTROL, b'PONG')]
    assert decoder.bad_frames == 1


def test_frame_encoder_uses_ccitt_false_checksum():
    decoder = FrameDecoder()
    assert decoder.feed(encode_frame(CHANNEL_CONTROL, b'123456789')) == [
        (CHANNEL_CONTROL, b'123456789')]
    assert int.from_bytes(encode_frame(
        CHANNEL_CONTROL, b'123456789')[-2:], 'little') == 0x58ae


def test_frame_encoder_rejects_oversized_payload():
    with pytest.raises(ValueError, match='exceeds'):
        encode_frame(CHANNEL_LIDAR, b'x' * 181)


def test_pty_path_is_a_symlink_and_mux_bytes_reach_slave(tmp_path):
    endpoint = PtySerial(str(tmp_path / 'lidar'))
    try:
        assert endpoint.link_path.is_symlink()
        assert os.readlink(endpoint.link_path) == endpoint.slave_path
        endpoint.forward_to_slave(b'\xa5\x20', None)
        assert os.read(endpoint.slave_fd, 2) == b'\xa5\x20'
    finally:
        endpoint.close()
    assert not (tmp_path / 'lidar').is_symlink()
