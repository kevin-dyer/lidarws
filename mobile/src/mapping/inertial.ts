import {
  applyRot,
  expSO3,
  logSO3,
  mulMat,
  norm,
  orthonormalize,
  rotationBetween,
  transpose,
  type Mat3,
  type Vec3,
} from './se3';

export type PhoneMount = 'portrait' | 'landscape-left' | 'landscape-right';

export interface ImuSample {
  /** Seconds on any monotonic clock shared with the lidar timestamps. */
  t: number;
  /** Angular rate about the device x, y, z axes in deg/s. */
  gyroDegS: Vec3;
  /** Accelerometer reading including gravity (m/s²) in device axes; points up when the phone is still. */
  accel: Vec3;
}

const G = 9.80665;

/** Expo's native DeviceMotion implementations use different rotation-rate field orders. */
export function deviceMotionGyro(
  rate: { alpha: number; beta: number; gamma: number },
  platform: string,
): Vec3 {
  if (platform === 'android') return [rate.alpha, rate.beta, rate.gamma];
  if (platform === 'ios') return [rate.gamma, rate.beta, rate.alpha];
  return [rate.beta, rate.gamma, rate.alpha];
}

/**
 * Rotation from phone axes (x right, y up the screen, z out of the screen) to scanner axes
 * (x forward, y left, z up) for a phone mounted on the scanner front with its back camera facing forward.
 */
export function mountRotation(mount: PhoneMount): Mat3 {
  switch (mount) {
    case 'portrait':
      return [0, 0, -1, -1, 0, 0, 0, 1, 0];
    case 'landscape-left':
      return [0, 0, -1, 0, 1, 0, 1, 0, 0];
    case 'landscape-right':
      return [0, 0, -1, 0, -1, 0, -1, 0, 0];
  }
}

interface Stamped {
  t: number;
  R: Mat3;
}

/**
 * Gyro-integrated scanner orientation in a gravity-aligned world frame (z up), with accelerometer tilt
 * correction. Yaw is relative to the start of the session and drifts slowly; lidar matching removes that drift.
 */
export class ImuOrientation {
  private readonly mount: Mat3;
  private R: Mat3 | null = null;
  private lastT = 0;
  // Platforms disagree on the sign of "accelerationIncludingGravity"; the first sample settles it,
  // since the scanner is always held upright (scanner z must read as up).
  private accelSign = 1;
  private readonly history: Stamped[] = [];

  constructor(mount: PhoneMount, private readonly historySeconds = 6, private readonly tiltTauSeconds = 1.5) {
    this.mount = mountRotation(mount);
  }

  get ready(): boolean {
    return this.R !== null;
  }

  get orientation(): Mat3 | null {
    return this.R;
  }

  update(sample: ImuSample): void {
    let specific = applyRot(this.mount, sample.accel);
    const magnitude = norm(specific);
    if (this.R === null) {
      if (magnitude < 0.5 * G) return;
      this.accelSign = specific[2] < 0 ? -1 : 1;
      specific = [specific[0] * this.accelSign, specific[1] * this.accelSign, specific[2] * this.accelSign];
      const up: Vec3 = [specific[0] / magnitude, specific[1] / magnitude, specific[2] / magnitude];
      this.R = rotationBetween(up, [0, 0, 1]);
      this.lastT = sample.t;
      this.record(sample.t);
      return;
    }

    specific = [specific[0] * this.accelSign, specific[1] * this.accelSign, specific[2] * this.accelSign];
    const dt = sample.t - this.lastT;
    this.lastT = sample.t;
    if (dt <= 0 || dt > 0.5) return;

    const rad = Math.PI / 180;
    const omega = applyRot(this.mount, [
      sample.gyroDegS[0] * rad,
      sample.gyroDegS[1] * rad,
      sample.gyroDegS[2] * rad,
    ]);
    let R = mulMat(this.R, expSO3([omega[0] * dt, omega[1] * dt, omega[2] * dt]));

    // Pull the estimated "up" toward the measured one while the device is not accelerating hard.
    if (magnitude > 0.85 * G && magnitude < 1.15 * G) {
      const up = applyRot(R, [specific[0] / magnitude, specific[1] / magnitude, specific[2] / magnitude]);
      const gain = 1 - Math.exp(-dt / this.tiltTauSeconds);
      // up x z: the world-frame rotation that swings the estimated up vector onto +z.
      R = mulMat(expSO3([up[1] * gain, -up[0] * gain, 0]), R);
    }
    this.R = orthonormalize(R);
    this.record(sample.t);
  }

  private record(t: number): void {
    if (!this.R) return;
    this.history.push({ t, R: this.R });
    const cutoff = t - this.historySeconds;
    while (this.history.length > 2 && this.history[0].t < cutoff) this.history.shift();
  }

  /** Orientation at time `t`, interpolated between samples; null before the filter has started. */
  orientationAt(t: number): Mat3 | null {
    const h = this.history;
    if (h.length === 0) return null;
    if (t <= h[0].t) return h[0].R;
    const last = h[h.length - 1];
    if (t >= last.t) return last.R;
    let lo = 0;
    let hi = h.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (h[mid].t <= t) lo = mid;
      else hi = mid;
    }
    const span = h[hi].t - h[lo].t;
    const f = span > 0 ? (t - h[lo].t) / span : 0;
    const delta = logSO3(mulMat(transpose(h[lo].R), h[hi].R));
    return mulMat(h[lo].R, expSO3([delta[0] * f, delta[1] * f, delta[2] * f]));
  }
}
