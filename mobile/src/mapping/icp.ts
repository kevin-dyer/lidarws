import type { VoxelMap } from './voxelMap';
import {
  cross,
  expSO3,
  logSO3,
  mulMat,
  orthonormalize,
  solveLinear,
  symmetricEigen,
  transformPoint,
  transpose,
  type Mat3,
  type Pose,
  type Vec3,
} from './se3';

export interface IcpOptions {
  iterations: number;
  /** Largest point-to-plane distance still treated as a match. */
  gateM: number;
  /** Weight of the IMU rotation prior relative to one match (scaled by match count). */
  rotationPriorWeight: number;
  /** Weight of the motion-model translation prior relative to one match. */
  translationPriorWeight: number;
}

export interface IcpResult {
  pose: Pose;
  matches: number;
  rmse: number;
  /** Smallest eigen-direction support, 0..1; low values mean the geometry barely constrains some direction. */
  degeneracy: number;
}

export const DEFAULT_ICP: IcpOptions = {
  iterations: 12,
  gateM: 0.35,
  rotationPriorWeight: 0.15,
  translationPriorWeight: 0.01,
};

/**
 * Point-to-plane scan-to-map registration. `initial` is the starting guess, and `prior` (IMU rotation +
 * motion-model translation) regularises the solution so poorly constrained directions follow the inertial estimate.
 */
export function registerScan(
  points: readonly Vec3[],
  map: VoxelMap,
  initial: Pose,
  prior: Pose,
  options: IcpOptions = DEFAULT_ICP,
): IcpResult {
  let pose: Pose = { R: [...initial.R] as Mat3, t: [...initial.t] as Vec3 };
  let matches = 0;
  let rmse = Infinity;
  let degeneracy = 0;

  for (let iteration = 0; iteration < options.iterations; iteration += 1) {
    const H = new Array<number>(36).fill(0);
    const g = new Array<number>(6).fill(0);
    let count = 0;
    let sumSq = 0;

    for (const local of points) {
      const world = transformPoint(pose, local);
      const plane = map.planeAt(world);
      if (!plane) continue;
      const n = plane.normal;
      const r =
        n[0] * (world[0] - plane.centroid[0]) +
        n[1] * (world[1] - plane.centroid[1]) +
        n[2] * (world[2] - plane.centroid[2]);
      const absR = Math.abs(r);
      if (absR > options.gateM) continue;
      // Huber weight keeps outliers from dragging the fit.
      const w = absR < 0.05 ? 1 : 0.05 / absR;
      const pxn = cross(world, n);
      const J = [pxn[0], pxn[1], pxn[2], n[0], n[1], n[2]];
      for (let i = 0; i < 6; i += 1) {
        g[i] += w * J[i] * r;
        for (let j = i; j < 6; j += 1) H[i * 6 + j] += w * J[i] * J[j];
      }
      count += 1;
      sumSq += r * r;
    }

    matches = count;
    rmse = count > 0 ? Math.sqrt(sumSq / count) : Infinity;
    if (count < 30) break;
    for (let i = 0; i < 6; i += 1) for (let j = 0; j < i; j += 1) H[i * 6 + j] = H[j * 6 + i];

    degeneracy = translationSupport(H);

    // Prior terms: rotation error log(R * Rprior^T) and translation error t - tprior.
    const eRot = logSO3(mulMat(pose.R, transpose(prior.R)));
    const wRot = options.rotationPriorWeight * count;
    const wTrans = options.translationPriorWeight * count;
    for (let i = 0; i < 3; i += 1) {
      H[i * 6 + i] += wRot;
      g[i] += wRot * eRot[i];
      H[(i + 3) * 6 + i + 3] += wTrans;
      g[i + 3] += wTrans * (pose.t[i] - prior.t[i]);
    }

    // Levenberg damping keeps the solve stable when geometry is nearly degenerate.
    const trace = H[0] + H[7] + H[14] + H[21] + H[28] + H[35];
    for (let i = 0; i < 6; i += 1) H[i * 6 + i] += 1e-6 * trace;
    const step = solveLinear(H, g.map((value) => -value), 6);
    if (!step) break;

    const dRot: Vec3 = [step[0], step[1], step[2]];
    const dR = expSO3(dRot);
    const R = orthonormalize(mulMat(dR, pose.R));
    const rotatedT = [
      dR[0] * pose.t[0] + dR[1] * pose.t[1] + dR[2] * pose.t[2],
      dR[3] * pose.t[0] + dR[4] * pose.t[1] + dR[5] * pose.t[2],
      dR[6] * pose.t[0] + dR[7] * pose.t[1] + dR[8] * pose.t[2],
    ];
    pose = { R, t: [rotatedT[0] + step[3], rotatedT[1] + step[4], rotatedT[2] + step[5]] };

    const rotMag = Math.hypot(step[0], step[1], step[2]);
    const transMag = Math.hypot(step[3], step[4], step[5]);
    if (rotMag < 1e-4 && transMag < 1e-4) break;
  }

  return { pose, matches, rmse, degeneracy };
}

/** Relative support of the weakest translation direction (0 = free to slide, 1 = isotropic). */
function translationSupport(H: number[]): number {
  const m = [H[21], H[22], H[23], H[27], H[28], H[29], H[33], H[34], H[35]];
  const trace = m[0] + m[4] + m[8];
  if (trace <= 0) return 0;
  const { values } = symmetricEigen([m[0], m[1], m[2], m[4], m[5], m[8]]);
  return Math.max(0, values[0]) / (trace / 3);
}
