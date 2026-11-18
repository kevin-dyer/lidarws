import { symmetricEigen, type Vec3 } from './se3';

interface Cell {
  n: number;
  s: [number, number, number];
  ss: [number, number, number, number, number, number];
  plane: { n: Vec3; c: Vec3 } | null | undefined;
}

export interface MapPlane {
  normal: Vec3;
  centroid: Vec3;
}

const KEY_OFFSET = 65536;
const KEY_SPAN = 131072;

function keyOf(ix: number, iy: number, iz: number): number {
  return ((ix + KEY_OFFSET) * KEY_SPAN + (iy + KEY_OFFSET)) * KEY_SPAN + (iz + KEY_OFFSET);
}

/** Smallest points-per-cell fit and flatness limits for treating a cell as a plane. */
const MIN_PLANE_POINTS = 6;
const MAX_PLANE_THICKNESS_RATIO = 0.08;
const MIN_PLANE_WIDTH_RATIO = 0.1;

/**
 * Sparse voxel map. Plane cells (centroid + normal from the covariance) are what scan-to-map matching uses;
 * a separate, finer one-point-per-cell cloud is kept for display and export.
 */
export class VoxelMap {
  private readonly cells = new Map<number, Cell>();
  private readonly cloud = new Map<number, Vec3>();

  constructor(
    readonly planeResolution = 0.5,
    readonly cloudResolution = 0.06,
    private readonly maxCloudPoints = 600000,
  ) {}

  get planeCount(): number {
    return this.cells.size;
  }

  get pointCount(): number {
    return this.cloud.size;
  }

  insert(points: readonly Vec3[]): void {
    const pr = 1 / this.planeResolution;
    const cr = 1 / this.cloudResolution;
    for (const p of points) {
      const key = keyOf(Math.floor(p[0] * pr), Math.floor(p[1] * pr), Math.floor(p[2] * pr));
      let cell = this.cells.get(key);
      if (!cell) {
        cell = { n: 0, s: [0, 0, 0], ss: [0, 0, 0, 0, 0, 0], plane: undefined };
        this.cells.set(key, cell);
      }
      cell.n += 1;
      cell.s[0] += p[0];
      cell.s[1] += p[1];
      cell.s[2] += p[2];
      cell.ss[0] += p[0] * p[0];
      cell.ss[1] += p[0] * p[1];
      cell.ss[2] += p[0] * p[2];
      cell.ss[3] += p[1] * p[1];
      cell.ss[4] += p[1] * p[2];
      cell.ss[5] += p[2] * p[2];
      cell.plane = undefined;

      if (this.cloud.size < this.maxCloudPoints) {
        const ckey = keyOf(Math.floor(p[0] * cr), Math.floor(p[1] * cr), Math.floor(p[2] * cr));
        if (!this.cloud.has(ckey)) this.cloud.set(ckey, [p[0], p[1], p[2]]);
      }
    }
  }

  planeAt(p: Vec3): MapPlane | null {
    const pr = 1 / this.planeResolution;
    const cell = this.cells.get(keyOf(Math.floor(p[0] * pr), Math.floor(p[1] * pr), Math.floor(p[2] * pr)));
    if (!cell || cell.n < MIN_PLANE_POINTS) return null;
    if (cell.plane === undefined) cell.plane = this.fit(cell);
    return cell.plane ? { normal: cell.plane.n, centroid: cell.plane.c } : null;
  }

  private fit(cell: Cell): { n: Vec3; c: Vec3 } | null {
    const inv = 1 / cell.n;
    const c: Vec3 = [cell.s[0] * inv, cell.s[1] * inv, cell.s[2] * inv];
    const cov: [number, number, number, number, number, number] = [
      cell.ss[0] * inv - c[0] * c[0],
      cell.ss[1] * inv - c[0] * c[1],
      cell.ss[2] * inv - c[0] * c[2],
      cell.ss[3] * inv - c[1] * c[1],
      cell.ss[4] * inv - c[1] * c[2],
      cell.ss[5] * inv - c[2] * c[2],
    ];
    const { values, vectors } = symmetricEigen(cov);
    const thickness = Math.sqrt(Math.max(values[0], 0));
    const width = Math.sqrt(Math.max(values[1], 0));
    if (thickness > MAX_PLANE_THICKNESS_RATIO * this.planeResolution) return null;
    if (width < MIN_PLANE_WIDTH_RATIO * this.planeResolution) return null;
    return { n: [vectors[0], vectors[1], vectors[2]], c };
  }

  /** Display/export points, evenly strided down to at most `max`. */
  getPoints(max = Infinity): Vec3[] {
    const all = Array.from(this.cloud.values());
    if (all.length <= max) return all;
    const step = all.length / max;
    const out: Vec3[] = [];
    for (let i = 0; i < max; i += 1) out.push(all[Math.floor(i * step)]);
    return out;
  }
}

/** Centroid of the points in each voxel of the given size. */
export function voxelDownsample(points: readonly Vec3[], size: number): Vec3[] {
  const inv = 1 / size;
  const seen = new Map<number, { sum: Vec3; n: number }>();
  for (const p of points) {
    const key = keyOf(Math.floor(p[0] * inv), Math.floor(p[1] * inv), Math.floor(p[2] * inv));
    const entry = seen.get(key);
    if (entry) {
      entry.sum[0] += p[0];
      entry.sum[1] += p[1];
      entry.sum[2] += p[2];
      entry.n += 1;
    } else {
      seen.set(key, { sum: [p[0], p[1], p[2]], n: 1 });
    }
  }
  return Array.from(seen.values(), ({ sum, n }): Vec3 => [sum[0] / n, sum[1] / n, sum[2] / n]);
}
