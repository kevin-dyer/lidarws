import { Buffer } from 'buffer';
import { encodeImage, encodeImu, encodeTimedLidar, fragmentMessage } from './desktopProtocol';

it('encodes timed points in FLU with nanosecond offsets', () => {
  const packet = encodeTimedLidar([
    { point: [1, 2, 3], time: 100 },
    { point: [4, 5, 6], time: 100.02 },
  ]);
  expect(packet.readUInt32LE(0)).toBe(6);
  expect(packet.readDoubleLE(4)).toBe(100);
  expect(packet.readUInt32LE(12)).toBe(2);
  expect([packet.readFloatLE(16), packet.readFloatLE(20), packet.readFloatLE(24)]).toEqual([1, -2, -3]);
  expect(packet.readUInt32LE(44)).toBe(20_000_000);
  expect(() => encodeTimedLidar([{ point: [0, 0, 0], time: 2 }, { point: [0, 0, 0], time: 1 }])).toThrow();
});

it('encodes image and IMU messages and fragments large payloads', () => {
  const image = encodeImage(10, Buffer.alloc(2500, 42));
  const fragments = fragmentMessage(image, 123, 456);
  expect(fragments).toHaveLength(3);
  expect(fragments[0].readUInt32LE(0)).toBe(4);
  expect(fragments[0].readUInt32LE(16)).toBe(3);
  expect(Buffer.concat(fragments.map((fragment) => fragment.subarray(24)))).toEqual(image);
  const imu = encodeImu(10, [1, 2, 3], [4, 5, 6]);
  expect(imu.readUInt32LE(0)).toBe(3);
  expect(imu.readFloatLE(32)).toBe(6);
});
