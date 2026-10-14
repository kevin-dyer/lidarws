import type { Ray } from './protocol';

export type Point3 = [number, number, number];

export function raysToWorldPoints(
  rays: Ray[],
  tiltDeg: number,
  azimuthOffsetDeg = 90,
  minRangeM = 0.05,
  maxRangeM = 30,
): Point3[] {
  const tilt = (tiltDeg * Math.PI) / 180;
  const cosTilt = Math.cos(tilt);
  const sinTilt = Math.sin(tilt);
  const points: Point3[] = [];
  for (const ray of rays) {
    if (!Number.isFinite(ray.rangeM) || ray.rangeM < minRangeM || ray.rangeM > maxRangeM) {
      continue;
    }
    const azimuth = ((ray.azimuthDeg + azimuthOffsetDeg) * Math.PI) / 180;
    const localX = ray.rangeM * Math.cos(azimuth);
    const localY = ray.rangeM * Math.sin(azimuth);
    const world = [cosTilt * localX, localY, sinTilt * localX] as const;
    if (world.every(Number.isFinite)) {
      points.push([world[0], world[1], world[2]]);
    }
  }
  return points;
}

export function encodeAsciiPly(points: Point3[]): string {
  const header = [
    'ply',
    'format ascii 1.0',
    'comment Tilt Lidar point cloud; final post-processing may be done separately',
    `element vertex ${points.length}`,
    'property double x',
    'property double y',
    'property double z',
    'end_header',
  ].join('\n');
  return `${header}\n${points.map(([x, y, z]) => `${x.toFixed(6)} ${y.toFixed(6)} ${z.toFixed(6)}`).join('\n')}\n`;
}

/** Parses an ASCII PLY produced by encodeAsciiPly, keeping at most `max` evenly spaced points. */
export function parseAsciiPly(text: string, max = Infinity): Point3[] {
  const bodyStart = text.indexOf('end_header');
  if (bodyStart < 0) return [];
  const lines = text.slice(text.indexOf('\n', bodyStart) + 1).split('\n');
  const total = lines.filter((line) => line.length > 0).length;
  const step = Math.max(1, total / max);
  const points: Point3[] = [];
  for (let index = 0; index < total; index += step) {
    const [x, y, z] = lines[Math.floor(index)].split(' ').map(Number);
    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) points.push([x, y, z]);
  }
  return points;
}
