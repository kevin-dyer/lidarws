"""Expose the Nano ESP32 BLE UART characteristics as Linux PTY serial ports."""
import argparse
import asyncio
from collections import deque
import errno
import os
from pathlib import Path
import pty
import sys
import tty

BLE_SERVICE_UUID = '7f510000-1b15-4f2d-9a4c-8b47a2d50000'
TILT_RX_UUID = '7f510001-1b15-4f2d-9a4c-8b47a2d50000'
TILT_TX_UUID = '7f510002-1b15-4f2d-9a4c-8b47a2d50000'
LIDAR_RX_UUID = '7f510003-1b15-4f2d-9a4c-8b47a2d50000'
LIDAR_TX_UUID = '7f510004-1b15-4f2d-9a4c-8b47a2d50000'

MAX_BLE_WRITE_SIZE = 180
MAX_PTY_BACKLOG = 1024 * 1024


class PtySerial:
    """A persistent PTY pair whose slave is exposed at a stable symlink."""

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

    def send_to_slave(self, data, loop):
        """Deliver BLE notification bytes to the PTY slave without blocking."""
        self._loop = loop
        self._pending.append(memoryview(data))
        self._pending_size += len(data)
        if self._pending_size > MAX_PTY_BACKLOG:
            raise BufferError('BLE-to-PTY backlog exceeded 1 MiB')
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
                raise OSError('PTY accepted zero bytes from BLE notification')
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
        """Close the PTY and remove only the symlink created by this instance."""
        if self._writer_registered and self._loop is not None:
            self._loop.remove_writer(self.master_fd)
            self._writer_registered = False
        for fd_name in ('master_fd', 'slave_fd'):
            fd = getattr(self, fd_name, None)
            if fd is not None:
                os.close(fd)
                setattr(self, fd_name, None)
        if self.link_path.is_symlink():
            try:
                if os.readlink(self.link_path) == self.slave_path:
                    self.link_path.unlink()
            except FileNotFoundError:
                pass


async def _forward_pty_to_ble(endpoint, client, characteristic, stopped):
    loop = asyncio.get_running_loop()
    queue = asyncio.Queue(maxsize=512)

    def on_readable():
        while True:
            try:
                payload = os.read(endpoint.master_fd, 4096)
            except BlockingIOError:
                return
            if not payload:
                return
            try:
                queue.put_nowait(payload)
            except asyncio.QueueFull:
                stopped.set()
                print('PTY-to-BLE queue is full; stopping bridge', file=sys.stderr)
                return

    loop.add_reader(endpoint.master_fd, on_readable)
    try:
        while not stopped.is_set():
            get_task = asyncio.create_task(queue.get())
            stop_task = asyncio.create_task(stopped.wait())
            done, pending = await asyncio.wait(
                (get_task, stop_task), return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            if stop_task in done:
                break
            payload = get_task.result()
            chunk_size = max(1, min(
                MAX_BLE_WRITE_SIZE, int(client.mtu_size) - 3))
            for offset in range(0, len(payload), chunk_size):
                await client.write_gatt_char(
                    characteristic,
                    payload[offset:offset + chunk_size],
                    response=False,
                )
    finally:
        loop.remove_reader(endpoint.master_fd)


async def _run_connected(client, lidar, tilt, disconnected):
    loop = asyncio.get_running_loop()
    stopped = asyncio.Event()

    def lidar_notify(_, data):
        try:
            lidar.send_to_slave(data, loop)
        except (BufferError, OSError) as exc:
            print('LiDAR BLE notification forwarding failed: %s' % exc,
                  file=sys.stderr)
            stopped.set()

    def tilt_notify(_, data):
        try:
            tilt.send_to_slave(data, loop)
        except (BufferError, OSError) as exc:
            print('Tilt BLE notification forwarding failed: %s' % exc,
                  file=sys.stderr)
            stopped.set()

    await client.start_notify(LIDAR_TX_UUID, lidar_notify)
    await client.start_notify(TILT_TX_UUID, tilt_notify)
    print('BLE_READY', flush=True)

    tasks = [
        asyncio.create_task(_forward_pty_to_ble(
            lidar, client, LIDAR_RX_UUID, stopped)),
        asyncio.create_task(_forward_pty_to_ble(
            tilt, client, TILT_RX_UUID, stopped)),
        asyncio.create_task(disconnected.wait()),
        asyncio.create_task(stopped.wait()),
    ]
    done, pending = await asyncio.wait(
        tasks, return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()
    await asyncio.gather(*pending, return_exceptions=True)
    for task in done:
        if task.cancelled():
            continue
        exception = task.exception()
        if exception is not None:
            raise exception


async def bridge(device_name, lidar, tilt):
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError

    while True:
        devices = await BleakScanner.discover(timeout=5.0)
        device = next(
            (candidate for candidate in devices
             if candidate.name == device_name),
            None,
        )
        if device is None:
            print('Waiting for BLE device %r' % device_name, file=sys.stderr)
            await asyncio.sleep(2.0)
            continue

        disconnected = asyncio.Event()
        try:
            async with BleakClient(
                    device,
                    disconnected_callback=lambda _: disconnected.set()) as client:
                print('Connected to %s' % device_name, flush=True)
                await _run_connected(client, lidar, tilt, disconnected)
        except (BleakError, OSError, asyncio.TimeoutError) as exc:
            print('BLE connection failed: %s' % exc, file=sys.stderr)
        await asyncio.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-name', default='TiltLidarBridge')
    parser.add_argument('--lidar-port', default='/tmp/tilt_lidar_ble/lidar')
    parser.add_argument('--tilt-port', default='/tmp/tilt_lidar_ble/tilt')
    args = parser.parse_args()
    if (Path(args.lidar_port).expanduser().absolute() ==
            Path(args.tilt_port).expanduser().absolute()):
        parser.error('--lidar-port and --tilt-port must be different paths')

    lidar = PtySerial(args.lidar_port)
    try:
        tilt = PtySerial(args.tilt_port)
    except Exception:
        lidar.close()
        raise
    print('PTY_READY lidar=%s tilt=%s' % (
        args.lidar_port, args.tilt_port), flush=True)
    try:
        asyncio.run(bridge(args.device_name, lidar, tilt))
    finally:
        lidar.close()
        tilt.close()


if __name__ == '__main__':
    main()
