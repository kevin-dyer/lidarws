import type { Point3 } from './pointCloud';

export interface ProjectedPoint {
  x: number;
  y: number;
  band: number;
}

export const HEIGHT_BANDS = 6;

/** Keeps at most `max` points by uniform striding. */
export function decimate(points: readonly Point3[], max: number): Point3[] {
  if (points.length <= max) return points.slice();
  const step = points.length / max;
  const out: Point3[] = [];
  for (let i = 0; i < max; i += 1) out.push(points[Math.floor(i * step)]);
  return out;
}

/**
 * Orbit-camera orthographic projection. Scanner frame: x forward, y left, z up.
 * Output coordinates are normalised to roughly [-1, 1] and bucketed by height.
 */
export function projectPoints(
  points: readonly Point3[],
  yawRad: number,
  pitchRad: number,
  radius: number,
): ProjectedPoint[] {
  const cy = Math.cos(yawRad);
  const sy = Math.sin(yawRad);
  const cp = Math.cos(pitchRad);
  const sp = Math.sin(pitchRad);
  const scale = 1 / Math.max(radius, 0.1);
  let minZ = Infinity;
  let maxZ = -Infinity;
  for (const p of points) {
    if (p[2] < minZ) minZ = p[2];
    if (p[2] > maxZ) maxZ = p[2];
  }
  const zRange = Math.max(maxZ - minZ, 0.001);
  return points.map(([x, y, z]) => {
    const rx = x * cy - y * sy;
    const ry = x * sy + y * cy;
    const screenX = ry;
    const screenY = z * cp - rx * sp;
    const band = Math.min(HEIGHT_BANDS - 1, Math.floor(((z - minZ) / zRange) * HEIGHT_BANDS));
    return { x: screenX * scale, y: screenY * scale, band };
  });
}
