"""Tests for the BLE-to-PTY transport used by the launch file."""
import asyncio
import os

from tilt_coordinator.ble_serial_bridge import (
    PtySerial,
    _forward_pty_to_ble,
)


def test_pty_path_is_a_symlink_and_ble_bytes_reach_slave(tmp_path):
    endpoint = PtySerial(str(tmp_path / 'lidar'))
    loop = asyncio.new_event_loop()
    try:
        assert endpoint.link_path.is_symlink()
        assert os.readlink(endpoint.link_path) == endpoint.slave_path
        endpoint.send_to_slave(b'\xa5\x20', loop)
        assert os.read(endpoint.slave_fd, 2) == b'\xa5\x20'
    finally:
        endpoint.close()
        loop.close()
    assert not (tmp_path / 'lidar').is_symlink()


def test_pty_serial_commands_are_chunked_for_ble(tmp_path):
    async def exercise():
        endpoint = PtySerial(str(tmp_path / 'tilt'))
        stopped = asyncio.Event()

        class FakeClient:
            mtu_size = 23

            def __init__(self):
                self.writes = []
                self.complete = asyncio.Event()

            async def write_gatt_char(self, characteristic, payload, response):
                assert response is False
                self.writes.append((characteristic, bytes(payload)))
                if sum(len(data) for _, data in self.writes) == 42:
                    self.complete.set()

        client = FakeClient()
        task = asyncio.create_task(_forward_pty_to_ble(
            endpoint, client, 'tilt-rx', stopped))
        try:
            os.write(endpoint.slave_fd, b'x' * 42)
            await asyncio.wait_for(client.complete.wait(), timeout=1.0)
            stopped.set()
            await task
            assert [len(data) for _, data in client.writes] == [20, 20, 2]
            assert all(name == 'tilt-rx' for name, _ in client.writes)
        finally:
            stopped.set()
            if not task.done():
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
            endpoint.close()

    asyncio.run(exercise())
