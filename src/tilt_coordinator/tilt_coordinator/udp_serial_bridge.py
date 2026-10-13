"""Multiplex LiDAR and tilt byte streams over UDP and expose serial PTYs."""
import argparse
import asyncio
import errno
import os
from collections import deque
from pathlib import Path
import pty
import socket
import tty

MAGIC = b'\xa5\x5a'
CHANNEL_LIDAR = 1
CHANNEL_TILT = 2
CHANNEL_CONTROL = 0x7f
MAX_PAYLOAD = 1190
MAX_DATAGRAM_SIZE = MAX_PAYLOAD + 7
MAX_PTY_BACKLOG = 1024 * 1024


def _crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (
                crc << 1) & 0xffff
    return crc


def encode_frame(channel, payload):
    """Encode one UDP frame with CRC-16/CCITT-FALSE."""
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('frame payload exceeds %d bytes' % MAX_PAYLOAD)
    header_and_payload = bytes((channel, len(payload) & 0xff,
                                len(payload) >> 8)) + payload
    checksum = _crc16(header_and_payload)
    return MAGIC + header_and_payload + checksum.to_bytes(2, 'little')


def decode_frame(packet):
    """Decode a complete datagram; reject truncation, extra bytes, and bad CRC."""
    if len(packet) < 7 or packet[:2] != MAGIC:
        raise ValueError('invalid frame header')
    channel = packet[2]
    length = packet[3] | packet[4] << 8
    if length > MAX_PAYLOAD or len(packet) != length + 7:
        raise ValueError('invalid frame length')
    payload = packet[5:5 + length]
    expected = int.from_bytes(packet[5 + length:7 + length], 'little')
    if _crc16(packet[2:5] + payload) != expected:
        raise ValueError('invalid frame checksum')
    return channel, payload


class PtySerial:
    """Create a stable symbolic link to a Linux PTY slave."""

    def __init__(self, link_path):
        self.link_path = Path(link_path).expanduser()
        self.link_path.parent.mkdir(parents=True, exist_ok=True)
        self.master_fd, self.slave_fd = pty.openpty()
        tty.setraw(self.slave_fd)
        os.set_blocking(self.master_fd, False)
        self.slave_path = os.ttyname(self.slave_fd)
        self._pending = deque()
        self._pending_size = 0
        self._writer_registered = False
        self._loop = None

        if self.link_path.exists() or self.link_path.is_symlink():
            if not self.link_path.is_symlink():
                self.close()
                raise FileExistsError(
                    'refusing to replace non-symlink PTY path %s' % self.link_path)
            if self.link_path.exists():
                self.close()
                raise FileExistsError(
                    'PTY path already points to an existing device: %s'
                    % self.link_path)
            self.link_path.unlink()
        self.link_path.symlink_to(self.slave_path)

    def forward_to_slave(self, data, loop):
        self._loop = loop
        self._pending.append(memoryview(data))
        self._pending_size += len(data)
        if self._pending_size > MAX_PTY_BACKLOG:
            raise BufferError('UDP-to-PTY backlog exceeded 1 MiB')
        self._flush_to_slave(loop)

    def _flush_to_slave(self, loop):
        while self._pending:
            chunk = self._pending[0]
            try:
                written = os.write(self.master_fd, chunk)
            except BlockingIOError:
                if not self._writer_registered:
                    loop.add_writer(self.master_fd, self._flush_to_slave, loop)
                    self._writer_registered = True
                return
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return
                raise
            if written == 0:
                raise OSError('PTY accepted zero bytes')
            self._pending_size -= written
            if written == len(chunk):
                self._pending.popleft()
            else:
                self._pending[0] = chunk[written:]
                return

        if self._writer_registered:
            loop.remove_writer(self.master_fd)
            self._writer_registered = False

    def close(self):
        if self._writer_registered and self._loop is not None:
            self._loop.remove_writer(self.master_fd)
            self._writer_registered = False
        for name in ('master_fd', 'slave_fd'):
            fd = getattr(self, name, None)
            if fd is not None:
                os.close(fd)
                setattr(self, name, None)
        if self.link_path.is_symlink():
            try:
                if os.readlink(self.link_path) == self.slave_path:
                    self.link_path.unlink()
            except FileNotFoundError:
                pass


class UdpMux:
    """Bridge UDP mux frames to independent LiDAR and tilt PTYs."""

    def __init__(self, lidar_pty, tilt_pty, board_ip, board_port, local_port):
        self.lidar_pty = lidar_pty
        self.tilt_pty = tilt_pty
        self.targets = [(board_ip, board_port)]
        if '255.255.255.255' in self.targets[0]:
            self.targets.append(('192.168.4.1', board_port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setblocking(False)
        self.socket.bind(('', local_port))
        self.peer = None
        self.ready = None
        self.bad_datagrams = 0

    def _on_udp_readable(self):
        loop = asyncio.get_running_loop()
        while True:
            try:
                packet, sender = self.socket.recvfrom(MAX_DATAGRAM_SIZE + 1)
            except BlockingIOError:
                return

            try:
                channel, payload = decode_frame(packet)
            except ValueError:
                self.bad_datagrams += 1
                continue

            if channel == CHANNEL_CONTROL and payload == b'PONG':
                self.peer = sender
                if not self.ready.done():
                    self.ready.set_result(True)
                continue
            if self.peer is None or sender != self.peer:
                continue
            if channel == CHANNEL_LIDAR:
                self.lidar_pty.forward_to_slave(payload, loop)
            elif channel == CHANNEL_TILT:
                self.tilt_pty.forward_to_slave(payload, loop)

    def _send_frame(self, channel, payload, target=None):
        destination = self.peer if target is None else target
        if destination is None:
            return
        self.socket.sendto(encode_frame(channel, payload), destination)

    def _on_pty_readable(self, endpoint, channel):
        while True:
            try:
                data = os.read(endpoint.master_fd, MAX_PAYLOAD)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return
                raise
            if not data:
                return
            self._send_frame(channel, data)

    async def run(self, discovery_timeout):
        loop = asyncio.get_running_loop()
        self.ready = loop.create_future()
        loop.add_reader(self.socket.fileno(), self._on_udp_readable)
        loop.add_reader(
            self.lidar_pty.master_fd,
            self._on_pty_readable,
            self.lidar_pty,
            CHANNEL_LIDAR,
        )
        loop.add_reader(
            self.tilt_pty.master_fd,
            self._on_pty_readable,
            self.tilt_pty,
            CHANNEL_TILT,
        )
        try:
            deadline = loop.time() + discovery_timeout
            while not self.ready.done():
                for target in self.targets:
                    self._send_frame(
                        CHANNEL_CONTROL, b'PING', target=target)
                remaining = deadline - loop.time()
                if remaining <= 0:
                    raise TimeoutError(
                        'Nano ESP32 did not answer UDP discovery at %s:%d. '
                        'Join the same Wi-Fi or connect to its fallback AP, '
                        'check the local firmware/tilt_stage/wifi_config.h '
                        'settings, and verify UDP is not blocked.' % (
                            self.targets[0][0], self.targets[0][1]))
                try:
                    await asyncio.wait_for(
                        asyncio.shield(self.ready), min(0.5, remaining))
                except asyncio.TimeoutError:
                    continue

            print('UDP_READY peer=%s:%d' % self.peer, flush=True)
            while True:
                self._send_frame(CHANNEL_CONTROL, b'PING')
                await asyncio.sleep(2.0)
        finally:
            loop.remove_reader(self.socket.fileno())
            loop.remove_reader(self.lidar_pty.master_fd)
            loop.remove_reader(self.tilt_pty.master_fd)
            self.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board-ip', default='255.255.255.255')
    parser.add_argument('--board-port', type=int, default=4210)
    parser.add_argument('--local-port', type=int, default=4211)
    parser.add_argument('--discovery-timeout', type=float, default=60.0)
    parser.add_argument('--lidar-pty', default='/tmp/tilt_lidar_udp/lidar')
    parser.add_argument('--tilt-pty', default='/tmp/tilt_lidar_udp/tilt')
    args = parser.parse_args()
    if (Path(args.lidar_pty).expanduser().absolute() ==
            Path(args.tilt_pty).expanduser().absolute()):
        parser.error('--lidar-pty and --tilt-pty must be different paths')
    if not (1 <= args.board_port <= 65535 and
            1 <= args.local_port <= 65535):
        parser.error('UDP ports must be between 1 and 65535')

    lidar = PtySerial(args.lidar_pty)
    try:
        tilt = PtySerial(args.tilt_pty)
    except Exception:
        lidar.close()
        raise

    mux = None
    try:
        mux = UdpMux(
            lidar, tilt, args.board_ip, args.board_port, args.local_port)
        print('PTY_READY lidar=%s tilt=%s' % (
            args.lidar_pty, args.tilt_pty), flush=True)
        asyncio.run(mux.run(args.discovery_timeout))
    finally:
        if mux is not None:
            mux.socket.close()
        lidar.close()
        tilt.close()


if __name__ == '__main__':
    main()
