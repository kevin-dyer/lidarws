export type Vec3 = [number, number, number];
/** Row-major 3x3 rotation matrix. */
export type Mat3 = [number, number, number, number, number, number, number, number, number];

export interface Pose {
  R: Mat3;
  t: Vec3;
}

export const IDENTITY_R: Mat3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

export function identityPose(): Pose {
  return { R: [...IDENTITY_R] as Mat3, t: [0, 0, 0] };
}

export function mulMat(a: Mat3, b: Mat3): Mat3 {
  const out = new Array<number>(9);
  for (let r = 0; r < 3; r += 1) {
    for (let c = 0; c < 3; c += 1) {
      out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
    }
  }
  return out as Mat3;
}

export function transpose(m: Mat3): Mat3 {
  return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
}

export function applyRot(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}

export function transformPoint(pose: Pose, p: Vec3): Vec3 {
  const r = applyRot(pose.R, p);
  return [r[0] + pose.t[0], r[1] + pose.t[1], r[2] + pose.t[2]];
}

export function composePoses(a: Pose, b: Pose): Pose {
  const t = applyRot(a.R, b.t);
  return { R: mulMat(a.R, b.R), t: [t[0] + a.t[0], t[1] + a.t[1], t[2] + a.t[2]] };
}

export function invertPose(p: Pose): Pose {
  const Rt = transpose(p.R);
  const t = applyRot(Rt, p.t);
  return { R: Rt, t: [-t[0], -t[1], -t[2]] };
}

export function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function norm(a: Vec3): number {
  return Math.sqrt(dot(a, a));
}

/** Rodrigues formula: rotation matrix for a rotation vector. */
export function expSO3(w: Vec3): Mat3 {
  const angle = norm(w);
  if (angle < 1e-12) {
    return [1, -w[2], w[1], w[2], 1, -w[0], -w[1], w[0], 1];
  }
  const k: Vec3 = [w[0] / angle, w[1] / angle, w[2] / angle];
  const s = Math.sin(angle);
  const c = Math.cos(angle);
  const v = 1 - c;
  return [
    c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s,
    k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s,
    k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v,
  ];
}

/** Inverse of expSO3. */
export function logSO3(m: Mat3): Vec3 {
  const cosAngle = Math.max(-1, Math.min(1, (m[0] + m[4] + m[8] - 1) / 2));
  const angle = Math.acos(cosAngle);
  const axis: Vec3 = [m[7] - m[5], m[2] - m[6], m[3] - m[1]];
  if (angle < 1e-8) return [axis[0] / 2, axis[1] / 2, axis[2] / 2];
  if (Math.PI - angle < 1e-5) {
    // Near pi the antisymmetric part vanishes; recover the axis from the symmetric part.
    const x = Math.sqrt(Math.max(0, (m[0] + 1) / 2));
    const y = Math.sqrt(Math.max(0, (m[4] + 1) / 2)) * (m[1] < 0 ? -1 : 1);
    const z = Math.sqrt(Math.max(0, (m[8] + 1) / 2)) * (m[2] < 0 ? -1 : 1);
    return [x * angle, y * angle, z * angle];
  }
  const scale = angle / (2 * Math.sin(angle));
  return [axis[0] * scale, axis[1] * scale, axis[2] * scale];
}

/** Re-orthonormalises a rotation matrix to stop numerical drift. */
export function orthonormalize(m: Mat3): Mat3 {
  let x: Vec3 = [m[0], m[1], m[2]];
  const xn = norm(x);
  x = [x[0] / xn, x[1] / xn, x[2] / xn];
  let y: Vec3 = [m[3], m[4], m[5]];
  const d = dot(x, y);
  y = [y[0] - d * x[0], y[1] - d * x[1], y[2] - d * x[2]];
  const yn = norm(y);
  y = [y[0] / yn, y[1] / yn, y[2] / yn];
  const z = cross(x, y);
  return [x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]];
}

/** Rotation taking unit vector `from` onto unit vector `to` by the shortest arc. */
export function rotationBetween(from: Vec3, to: Vec3): Mat3 {
  const axis = cross(from, to);
  const s = norm(axis);
  const c = dot(from, to);
  if (s < 1e-9) {
    if (c > 0) return [...IDENTITY_R] as Mat3;
    const helper: Vec3 = Math.abs(from[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    const perp = cross(from, helper);
    const pn = norm(perp);
    return expSO3([(perp[0] / pn) * Math.PI, (perp[1] / pn) * Math.PI, (perp[2] / pn) * Math.PI]);
  }
  const angle = Math.atan2(s, c);
  return expSO3([(axis[0] / s) * angle, (axis[1] / s) * angle, (axis[2] / s) * angle]);
}

/** Solves A x = b (n x n, row-major) by Gaussian elimination with partial pivoting. */
export function solveLinear(a: number[], b: number[], n: number): number[] | null {
  const m = a.slice();
  const x = b.slice();
  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(m[row * n + col]) > Math.abs(m[pivot * n + col])) pivot = row;
    }
    if (Math.abs(m[pivot * n + col]) < 1e-12) return null;
    if (pivot !== col) {
      for (let k = 0; k < n; k += 1) {
        const tmp = m[col * n + k];
        m[col * n + k] = m[pivot * n + k];
        m[pivot * n + k] = tmp;
      }
      const tmp = x[col];
      x[col] = x[pivot];
      x[pivot] = tmp;
    }
    for (let row = col + 1; row < n; row += 1) {
      const f = m[row * n + col] / m[col * n + col];
      if (f === 0) continue;
      for (let k = col; k < n; k += 1) m[row * n + k] -= f * m[col * n + k];
      x[row] -= f * x[col];
    }
  }
  for (let row = n - 1; row >= 0; row -= 1) {
    let sum = x[row];
    for (let k = row + 1; k < n; k += 1) sum -= m[row * n + k] * x[k];
    x[row] = sum / m[row * n + row];
  }
  return x;
}

/**
 * Eigen-decomposition of a symmetric 3x3 matrix given as [xx, xy, xz, yy, yz, zz] using Jacobi rotations.
 * Eigenvalues are ascending; row i of `vectors` is the eigenvector for values[i].
 */
export function symmetricEigen(cov: [number, number, number, number, number, number]): {
  values: Vec3;
  vectors: Mat3;
} {
  const a = [cov[0], cov[1], cov[2], cov[1], cov[3], cov[4], cov[2], cov[4], cov[5]];
  const v = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let sweep = 0; sweep < 12; sweep += 1) {
    const off = Math.abs(a[1]) + Math.abs(a[2]) + Math.abs(a[5]);
    if (off < 1e-14) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]] as const) {
      const apq = a[p * 3 + q];
      if (Math.abs(apq) < 1e-18) continue;
      const theta = (a[q * 3 + q] - a[p * 3 + p]) / (2 * apq);
      const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1);
      const s = t * c;
      for (let k = 0; k < 3; k += 1) {
        const akp = a[k * 3 + p];
        const akq = a[k * 3 + q];
        a[k * 3 + p] = c * akp - s * akq;
        a[k * 3 + q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k += 1) {
        const apk = a[p * 3 + k];
        const aqk = a[q * 3 + k];
        a[p * 3 + k] = c * apk - s * aqk;
        a[q * 3 + k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k += 1) {
        const vkp = v[k * 3 + p];
        const vkq = v[k * 3 + q];
        v[k * 3 + p] = c * vkp - s * vkq;
        v[k * 3 + q] = s * vkp + c * vkq;
      }
    }
  }
  const order = [0, 1, 2].sort((i, j) => a[i * 3 + i] - a[j * 3 + j]);
  const values: Vec3 = [a[order[0] * 4], a[order[1] * 4], a[order[2] * 4]];
  const vectors = order.flatMap((i) => [v[i], v[3 + i], v[6 + i]]) as Mat3;
  return { values, vectors };
}
