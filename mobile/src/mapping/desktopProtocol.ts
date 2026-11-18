import { Buffer } from 'buffer';
import type { Point3 } from '../scanner/pointCloud';
import { flipFrdFlu } from './frames';

export function encodeImu(timestamp: number, acceleration: Point3, angularVelocity: Point3): Buffer {
  const message = Buffer.alloc(36);
  message.writeUInt32LE(3, 0);
  message.writeDoubleLE(timestamp, 4);
  [...acceleration, ...angularVelocity].forEach((value, index) => message.writeFloatLE(value, 12 + index * 4));
  return message;
}

export function encodeImage(timestamp: number, jpeg: Uint8Array): Buffer {
  const header = Buffer.alloc(12);
  header.writeUInt32LE(5, 0);
  header.writeDoubleLE(timestamp, 4);
  return Buffer.concat([header, Buffer.from(jpeg)]);
}

export function encodeTimedLidar(points: Array<{ point: Point3; time: number }>): Buffer {
  if (!points.length) throw new Error('Cannot send an empty LiDAR scan.');
  const start = points[0].time;
  const message = Buffer.alloc(16 + points.length * 16);
  message.writeUInt32LE(6, 0);
  message.writeDoubleLE(start, 4);
  message.writeUInt32LE(points.length, 12);
  let last = -1;
  points.forEach(({ point, time }, index) => {
    const offsetNs = Math.round((time - start) * 1e9);
    if (!Number.isFinite(offsetNs) || offsetNs < last || offsetNs > 0xffffffff) {
      throw new Error('LiDAR timestamps are out of order or exceed the scan duration limit.');
    }
    last = offsetNs;
    const offset = 16 + index * 16;
    flipFrdFlu(point).forEach((coordinate, axis) => message.writeFloatLE(coordinate, offset + 4 * axis));
    message.writeUInt32LE(offsetNs, offset + 12);
  });
  return message;
}

export function fragmentMessage(message: Uint8Array, idLow: number, idHigh: number): Buffer[] {
  const size = 1200;
  if (message.length <= size) return [Buffer.from(message)];
  if (message.length > 10 * 1024 * 1024) throw new Error('Sensor message exceeds the desktop bridge limit.');
  const count = Math.ceil(message.length / size);
  return Array.from({ length: count }, (_, index) => {
    const part = message.subarray(index * size, (index + 1) * size);
    const header = Buffer.alloc(24);
    header.writeUInt32LE(4, 0);
    header.writeUInt32LE(idLow, 4);
    header.writeUInt32LE(idHigh, 8);
    header.writeUInt32LE(index, 12);
    header.writeUInt32LE(count, 16);
    header.writeUInt32LE(message.length, 20);
    return Buffer.concat([header, Buffer.from(part)]);
  });
}
