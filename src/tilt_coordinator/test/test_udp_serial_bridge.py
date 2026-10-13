"""Tests for UDP mux framing and PTY forwarding."""
import asyncio
import os
import socket

import pytest

from tilt_coordinator.udp_serial_bridge import (
    CHANNEL_CONTROL,
    CHANNEL_LIDAR,
    CHANNEL_TILT,
    PtySerial,
    UdpMux,
    decode_frame,
    encode_frame,
)


def test_frame_round_trip():
    frame = encode_frame(CHANNEL_LIDAR, b'\xa5\x20\x00\xff')
    assert decode_frame(frame) == (CHANNEL_LIDAR, b'\xa5\x20\x00\xff')


def test_control_frame_crc_matches_crc16_ccitt_false():
    frame = encode_frame(CHANNEL_CONTROL, b'123456789')
    assert int.from_bytes(frame[-2:], 'little') == 0x58ae
    assert decode_frame(frame) == (CHANNEL_CONTROL, b'123456789')


def test_decode_rejects_invalid_header_length_and_crc():
    frame = encode_frame(CHANNEL_TILT, b'T0.0\n')
    with pytest.raises(ValueError, match='header'):
        decode_frame(b'bad' + frame[3:])
    with pytest.raises(ValueError, match='length'):
        decode_frame(frame + b'extra')

    invalid_crc = bytearray(frame)
    invalid_crc[-1] ^= 1
    with pytest.raises(ValueError, match='checksum'):
        decode_frame(invalid_crc)


def test_encoder_rejects_oversized_udp_payload():
    with pytest.raises(ValueError, match='exceeds'):
        encode_frame(CHANNEL_LIDAR, b'x' * 1191)


def test_pty_path_is_a_symlink_and_udp_payload_reaches_slave(tmp_path):
    endpoint = PtySerial(str(tmp_path / 'lidar'))
    try:
        assert endpoint.link_path.is_symlink()
        assert os.readlink(endpoint.link_path) == endpoint.slave_path
        endpoint.forward_to_slave(b'\xa5\x20', None)
        assert os.read(endpoint.slave_fd, 2) == b'\xa5\x20'
    finally:
        endpoint.close()
    assert not (tmp_path / 'lidar').is_symlink()


def test_udp_mux_handshake_and_lidar_forwarding(tmp_path):
    async def exercise():
        lidar = PtySerial(str(tmp_path / 'lidar'))
        tilt = PtySerial(str(tmp_path / 'tilt'))
        mux = UdpMux(lidar, tilt, '127.0.0.1', 4210, 0)
        board = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        board.bind(('127.0.0.1', 0))
        loop = asyncio.get_running_loop()
        mux.ready = loop.create_future()
        loop.add_reader(mux.socket.fileno(), mux._on_udp_readable)
        try:
            host = mux.socket.getsockname()
            board.sendto(encode_frame(CHANNEL_CONTROL, b'PONG'), host)
            await asyncio.wait_for(mux.ready, timeout=1.0)
            assert mux.peer == board.getsockname()

            board.sendto(encode_frame(CHANNEL_LIDAR, b'\xa5\x20'), host)
            await asyncio.sleep(0.01)
            assert os.read(lidar.slave_fd, 2) == b'\xa5\x20'
        finally:
            loop.remove_reader(mux.socket.fileno())
            board.close()
            mux.socket.close()
            lidar.close()
            tilt.close()

    asyncio.run(exercise())
