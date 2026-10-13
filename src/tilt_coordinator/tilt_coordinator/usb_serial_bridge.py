"""Multiplex LiDAR and tilt byte streams over one USB serial connection."""
import argparse
import asyncio
import errno
import os
from collections import deque
from pathlib import Path
import pty
import tty

MAGIC = b'\xa5\x5a'
CHANNEL_LIDAR = 1
CHANNEL_TILT = 2
CHANNEL_CONTROL = 0x7f
MAX_PAYLOAD = 180
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
    """Encode one transport frame with CRC-16/CCITT-FALSE."""
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('frame payload exceeds %d bytes' % MAX_PAYLOAD)
    header_and_payload = bytes((channel, len(payload) & 0xff,
                                len(payload) >> 8)) + payload
    checksum = _crc16(header_and_payload)
    return MAGIC + header_and_payload + checksum.to_bytes(2, 'little')


class FrameDecoder:
    """Incremental decoder that drops frames with invalid length or CRC."""

    def __init__(self):
        self._buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                if self._buffer[-1:] == MAGIC[:1]:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return frames
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 7:
                return frames

            channel = self._buffer[2]
            length = self._buffer[3] | self._buffer[4] << 8
            if length > MAX_PAYLOAD:
                self.bad_frames += 1
                del self._buffer[0]
                continue

            frame_size = 7 + length
            if len(self._buffer) < frame_size:
                return frames

            payload = bytes(self._buffer[5:5 + length])
            expected_crc = int.from_bytes(
                self._buffer[5 + length:7 + length], 'little')
            if _crc16(bytes((channel, self._buffer[3], self._buffer[4])) +
                      payload) != expected_crc:
                self.bad_frames += 1
                del self._buffer[0]
                continue

            frames.append((channel, payload))
            del self._buffer[:frame_size]


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
            raise BufferError('USB-to-PTY backlog exceeded 1 MiB')
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


class UsbSerialMux:
    """Run the USB transport and expose LiDAR and tilt serial PTYs."""

    def __init__(self, serial_port, lidar_pty, tilt_pty):
        self.serial_port = serial_port
        self.lidar_pty = lidar_pty
        self.tilt_pty = tilt_pty
        self.decoder = FrameDecoder()
        self.ready = None
        self.bytes_received = 0
        self.frames_received = 0

    def _on_usb_readable(self):
        try:
            data = self.serial_port.read(4096)
        except OSError as exc:
            if not self.ready.done():
                self.ready.set_exception(exc)
            return
        self.bytes_received += len(data)
        frames = self.decoder.feed(data)
        self.frames_received += len(frames)
        for channel, payload in frames:
            if channel == CHANNEL_LIDAR:
                self.lidar_pty.forward_to_slave(payload, asyncio.get_running_loop())
            elif channel == CHANNEL_TILT:
                self.tilt_pty.forward_to_slave(payload, asyncio.get_running_loop())
            elif channel == CHANNEL_CONTROL and payload == b'PONG':
                if not self.ready.done():
                    self.ready.set_result(True)

    def _send_frame(self, channel, payload):
        packet = memoryview(encode_frame(channel, payload))
        while packet:
            written = self.serial_port.write(packet)
            if written <= 0:
                raise OSError('USB serial accepted zero bytes')
            packet = packet[written:]

    def _on_pty_readable(self, endpoint, channel):
        while True:
            try:
                data = os.read(endpoint.master_fd, 4096)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return
                raise
            if not data:
                return
            for offset in range(0, len(data), MAX_PAYLOAD):
                self._send_frame(channel, data[offset:offset + MAX_PAYLOAD])

    async def connect_and_run(self, port, baudrate):
        import serial

        self.ready = asyncio.get_running_loop().create_future()
        self.serial_port = serial.Serial(
            port=None,
            baudrate=baudrate,
            timeout=0,
            write_timeout=1,
            rtscts=False,
            dsrdtr=False,
        )
        self.serial_port.port = port
        self.serial_port.dtr = True
        self.serial_port.rts = False
        self.serial_port.open()
        await asyncio.sleep(2.0)
        loop = asyncio.get_running_loop()
        loop.add_reader(self.serial_port.fileno(), self._on_usb_readable)
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
            deadline = loop.time() + 20.0
            while not self.ready.done():
                self._send_frame(CHANNEL_CONTROL, b'PING')
                remaining = deadline - loop.time()
                if remaining <= 0:
                    raise TimeoutError(
                        'Nano ESP32 did not answer USB mux PING; '
                        'received %d bytes in %d valid mux frames '
                        '(%d invalid frames). Confirm that '
                        'firmware/tilt_stage/tilt_stage.ino is uploaded and '
                        'that this is the correct USB port.' % (
                            self.bytes_received,
                            self.frames_received,
                            self.decoder.bad_frames))
                try:
                    await asyncio.wait_for(
                        asyncio.shield(self.ready), min(0.5, remaining))
                except asyncio.TimeoutError:
                    continue
            print('USB_READY', flush=True)
            await asyncio.Future()
        finally:
            loop.remove_reader(self.serial_port.fileno())
            loop.remove_reader(self.lidar_pty.master_fd)
            loop.remove_reader(self.tilt_pty.master_fd)
            self.serial_port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--lidar-pty', default='/tmp/tilt_lidar_usb/lidar')
    parser.add_argument('--tilt-pty', default='/tmp/tilt_lidar_usb/tilt')
    args = parser.parse_args()
    if (Path(args.lidar_pty).expanduser().absolute() ==
            Path(args.tilt_pty).expanduser().absolute()):
        parser.error('--lidar-pty and --tilt-pty must be different paths')

    lidar = PtySerial(args.lidar_pty)
    try:
        tilt = PtySerial(args.tilt_pty)
    except Exception:
        lidar.close()
        raise

    mux = UsbSerialMux(None, lidar, tilt)
    print('PTY_READY lidar=%s tilt=%s' % (
        args.lidar_pty, args.tilt_pty), flush=True)
    try:
        asyncio.run(mux.connect_and_run(args.device_port, args.baud))
    finally:
        lidar.close()
        tilt.close()


if __name__ == '__main__':
    main()
