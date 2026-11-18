import { Buffer } from 'buffer';
import { DesktopStream } from './desktopStream';

type MessageHandler = (message: Uint8Array, info: { address: string; port: number }) => void;
let mockMessageHandler: MessageHandler | undefined;
const mockSend = (data: Uint8Array, _offset: number, _length: number, _port: number, _host: string, callback: (error?: Error) => void) => {
  callback();
  if (Buffer.from(data).toString() === 'LIVO_HELLO') {
    Promise.resolve().then(() => mockMessageHandler?.(Buffer.from('LIVO_OK'), { address: '192.168.1.50', port: 5005 }));
  }
};
const mockSocket = {
  bind: jest.fn((_port: number, _address: string, callback: (error?: Error) => void) => callback()),
  on: jest.fn((event: string, callback: MessageHandler) => {
    if (event === 'message') mockMessageHandler = callback;
  }),
  send: jest.fn(mockSend),
  close: jest.fn(),
};

jest.mock('react-native-udp', () => ({
  __esModule: true,
  default: { createSocket: () => mockSocket },
}));

beforeEach(() => {
  jest.clearAllMocks();
  mockMessageHandler = undefined;
  mockSocket.send.mockImplementation(mockSend);
});

it('waits for the desktop handshake and sends sensor data', async () => {
  const stream = new DesktopStream('192.168.1.50', jest.fn());
  await stream.open();
  await stream.send(Buffer.from([3, 0, 0, 0]), 'imu');
  expect(mockSocket.send).toHaveBeenCalledTimes(2);
  expect(mockSocket.send.mock.calls[1][4]).toBe('192.168.1.50');
  stream.close();
  expect(mockSocket.close).toHaveBeenCalledTimes(1);
});

it('rejects a socket bind error instead of starting the stream', async () => {
  mockSocket.bind.mockImplementationOnce((_port, _address, callback) => callback(new Error('bind failed')));
  const stream = new DesktopStream('192.168.1.50', jest.fn());
  await expect(stream.open()).rejects.toThrow('bind failed');
  expect(mockSocket.send).not.toHaveBeenCalled();
  stream.close();
});

it('reports when the desktop bridge stops acknowledging heartbeats', async () => {
  jest.useFakeTimers();
  try {
    const onError = jest.fn();
    const stream = new DesktopStream('192.168.1.50', onError);
    await stream.open();
    for (let i = 0; i < 3; i += 1) {
      mockSocket.send.mockImplementationOnce((_data, _offset, _length, _port, _host, callback) => callback());
    }
    await jest.advanceTimersByTimeAsync(12000);
    expect(onError).toHaveBeenCalledWith(expect.objectContaining({ message: 'Lost contact with the desktop bridge.' }));
    stream.close();
  } finally {
    jest.useRealTimers();
  }
});

it('sends overlapping LiDAR scans in order instead of failing on a transient backlog', async () => {
  const callbacks: Array<(error?: Error) => void> = [];
  const stream = new DesktopStream('192.168.1.50', jest.fn());
  await stream.open();
  mockSocket.send.mockImplementation((_data, _offset, _length, _port, _host, callback) => {
    callbacks.push(callback);
  });
  const first = stream.send(Buffer.from([6, 1]), 'lidar');
  const second = stream.send(Buffer.from([6, 2]), 'lidar');
  expect(mockSocket.send).toHaveBeenCalledTimes(2);
  callbacks.shift()?.();
  await first;
  expect(Buffer.from(mockSocket.send.mock.calls[2][0])).toEqual(Buffer.from([6, 2]));
  callbacks.shift()?.();
  await second;
  stream.close();
});

it('finishes every fragment of a scan before sending the next scan', async () => {
  const stream = new DesktopStream('192.168.1.50', jest.fn());
  await stream.open();
  await Promise.all([
    stream.send(Buffer.alloc(1201, 1), 'lidar'),
    stream.send(Buffer.alloc(1201, 2), 'lidar'),
  ]);
  const packets = mockSocket.send.mock.calls.slice(1).map(([data]) => Buffer.from(data));
  expect(packets.map((packet) => packet[24])).toEqual([1, 1, 2, 2]);
  expect(packets.map((packet) => packet.readUInt32LE(12))).toEqual([0, 1, 0, 1]);
  stream.close();
});

it('bounds the LiDAR backlog and rejects queued scans when closed', async () => {
  let callback: ((error?: Error) => void) | undefined;
  const stream = new DesktopStream('192.168.1.50', jest.fn());
  await stream.open();
  mockSocket.send.mockImplementation((_data, _offset, _length, _port, _host, done) => {
    callback = done;
  });
  const first = stream.send(Buffer.from([6, 0]), 'lidar');
  const queued = Array.from({ length: 7 }, (_, i) =>
    stream.send(Buffer.from([6, i + 1]), 'lidar').catch((error: Error) => error.message),
  );
  await expect(stream.send(Buffer.from([6, 8]), 'lidar')).rejects.toThrow('LiDAR queue is full');
  stream.close();
  expect(await Promise.all(queued)).toEqual(Array(7).fill('Desktop stream is closed.'));
  callback?.();
  await first;
  expect(mockSocket.send).toHaveBeenCalledTimes(2);
});

it('propagates a send failure to the active and queued scans', async () => {
  let callback: ((error?: Error) => void) | undefined;
  const stream = new DesktopStream('192.168.1.50', jest.fn());
  await stream.open();
  mockSocket.send.mockImplementation((_data, _offset, _length, _port, _host, done) => {
    callback = done;
  });
  const first = stream.send(Buffer.from([6, 0]), 'lidar').catch((error: Error) => error.message);
  const second = stream.send(Buffer.from([6, 1]), 'lidar').catch((error: Error) => error.message);
  callback?.(new Error('send failed'));
  expect(await Promise.all([first, second])).toEqual(['send failed', 'send failed']);
  expect(mockSocket.send).toHaveBeenCalledTimes(2);
  stream.close();
});
