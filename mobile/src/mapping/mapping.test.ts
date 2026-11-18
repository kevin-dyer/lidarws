import { deviceMotionGyro, ImuOrientation, mountRotation } from './inertial';
import { MappingEngine, DEFAULT_ENGINE_OPTIONS } from './odometry';
import { TiltTrack } from './tiltTrack';
import { minimumSweepPeriod, validateMappingSettings, DEFAULT_MAPPING_SETTINGS } from './settings';
import {
  applyRot,
  expSO3,
  logSO3,
  mulMat,
  symmetricEigen,
  transpose,
  type Mat3,
  type Vec3,
} from './se3';
import { raysToWorldPoints } from '../scanner/pointCloud';

const G = 9.80665;

describe('SO(3) helpers', () => {
  it('round-trips exp and log', () => {
    const w: Vec3 = [0.3, -0.5, 0.2];
    const back = logSO3(expSO3(w));
    back.forEach((value, index) => expect(value).toBeCloseTo(w[index], 9));
  });

  it('finds the thin direction of a flat covariance', () => {
    const { values, vectors } = symmetricEigen([1, 0, 0, 2, 0, 0.001]);
    expect(values[0]).toBeCloseTo(0.001, 9);
    expect(Math.abs(vectors[2])).toBeCloseTo(1, 9);
  });
});

describe('ImuOrientation', () => {
  it('maps native DeviceMotion rotation rates to device x, y, z axes', () => {
    expect(deviceMotionGyro({ alpha: 1, beta: 2, gamma: 3 }, 'android')).toEqual([1, 2, 3]);
    expect(deviceMotionGyro({ alpha: 1, beta: 2, gamma: 3 }, 'ios')).toEqual([3, 2, 1]);
  });

  it('aligns z up from gravity and follows a gyro yaw', () => {
    const mount = mountRotation('portrait');
    const imu = new ImuOrientation('portrait');
    // Scanner at rest, level: specific force is +z in the scanner frame; map it into device axes.
    const accel = applyRot(transpose(mount), [0, 0, G]);
    imu.update({ t: 0, gyroDegS: [0, 0, 0], accel });
    expect(imu.ready).toBe(true);
    const up = applyRot(imu.orientation as Mat3, [0, 0, 1]);
    expect(up[2]).toBeCloseTo(1, 6);

    // 10 deg/s about scanner z for 3 s.
    const gyro = applyRot(transpose(mount), [0, 0, 10]);
    for (let i = 1; i <= 150; i += 1) imu.update({ t: i * 0.02, gyroDegS: gyro, accel });
    const forward = applyRot(imu.orientation as Mat3, [1, 0, 0]);
    expect(Math.atan2(forward[1], forward[0]) * (180 / Math.PI)).toBeCloseTo(30, 1);
  });

  it('accepts accelerometers that report gravity with the opposite sign', () => {
    const mount = mountRotation('portrait');
    const imu = new ImuOrientation('portrait');
    const flipped = applyRot(transpose(mount), [0, 0, -G]);
    imu.update({ t: 0, gyroDegS: [0, 0, 0], accel: flipped });
    for (let i = 1; i <= 50; i += 1) imu.update({ t: i * 0.02, gyroDegS: [0, 0, 0], accel: flipped });
    const up = applyRot(imu.orientation as Mat3, [0, 0, 1]);
    expect(up[2]).toBeCloseTo(1, 6);
  });

  it('corrects a tilt error using gravity', () => {
    const mount = mountRotation('portrait');
    const imu = new ImuOrientation('portrait');
    const level = applyRot(transpose(mount), [0, 0, G]);
    imu.update({ t: 0, gyroDegS: [0, 0, 0], accel: level });
    // A gyro bias tilts the estimate; gravity keeps pulling it back.
    const bias = applyRot(transpose(mount), [0, 2, 0]);
    for (let i = 1; i <= 500; i += 1) imu.update({ t: i * 0.02, gyroDegS: bias, accel: level });
    const up = applyRot(imu.orientation as Mat3, [0, 0, 1]);
    expect(up[2]).toBeGreaterThan(0.995);
  });
});

describe('TiltTrack', () => {
  it('interpolates between samples', () => {
    const track = new TiltTrack();
    track.push(0, 0);
    track.push(1, 10);
    expect(track.angleAt(0.25)).toBeCloseTo(2.5, 9);
  });
});

describe('mapping settings', () => {
  it('accepts the defaults and rejects a sweep the stage cannot follow', () => {
    expect(validateMappingSettings(DEFAULT_MAPPING_SETTINGS)).toBeNull();
    expect(minimumSweepPeriod(-20, 20)).toBeCloseTo(2, 9);
    expect(validateMappingSettings({ ...DEFAULT_MAPPING_SETTINGS, sweepPeriodSeconds: 1 })).not.toBeNull();
    expect(validateMappingSettings({ ...DEFAULT_MAPPING_SETTINGS, minAngleDeg: 10, maxAngleDeg: 0 })).not.toBeNull();
  });
});

describe('MappingEngine on a simulated room', () => {
  const ROOM: Vec3 = [6, 5, 3];

  function castRange(origin: Vec3, dir: Vec3): number {
    let best = Infinity;
    for (let axis = 0; axis < 3; axis += 1) {
      if (Math.abs(dir[axis]) < 1e-9) continue;
      const wall = dir[axis] > 0 ? ROOM[axis] : 0;
      const d = (wall - origin[axis]) / dir[axis];
      if (d > 0 && d < best) best = d;
    }
    return best;
  }

  it('keeps tracking a stationary room after initialization with A1-rate revolutions', () => {
    const mount = mountRotation('portrait');
    const engine = new MappingEngine({ mount: 'portrait', ...DEFAULT_ENGINE_OPTIONS });
    const accel = applyRot(transpose(mount), [0, 0, G]);
    const origin: Vec3 = [2, 2.5, 1.2];
    let nextRev = 0.18;
    for (let step = 1; step <= 400; step += 1) {
      const t = step * 0.02;
      engine.addImu({ t, gyroDegS: [0, 0, 0], accel });
      if (t + 1e-9 < nextRev) continue;
      const phase = ((t - 0.09) % 5) / 5;
      const tilt = phase < 0.5 ? -20 + 80 * phase : 60 - 80 * phase;
      const rays = Array.from({ length: 360 }, (_, i) => ({ azimuthDeg: i - 90, rangeM: 1 }));
      const directions = raysToWorldPoints(rays, tilt);
      const points: Vec3[] = directions.map((direction, i) => {
        const range = castRange(origin, direction) + 0.01 * Math.sin(i * 19 + t * 7);
        return [direction[0] * range, direction[1] * range, direction[2] * range];
      });
      engine.addRevolution(t - 0.09, points);
      if (engine.getStats().batches > 1) expect(engine.getStats().state).toBe('tracking');
      nextRev += 0.18;
    }
    expect(engine.getStats().batches).toBeGreaterThan(1);
  });

  it('recovers tracking when short batches have too few usable lidar returns', () => {
    const mount = mountRotation('portrait');
    const engine = new MappingEngine({ mount: 'portrait', ...DEFAULT_ENGINE_OPTIONS });
    const accel = applyRot(transpose(mount), [0, 0, G]);
    const origin: Vec3 = [2, 2.5, 1.2];
    let nextRev = 0.18;
    let lost = false;
    let recovered = false;
    let seededPoints = 0;
    for (let step = 1; step <= 600; step += 1) {
      const t = step * 0.02;
      engine.addImu({ t, gyroDegS: [0, 0, 0], accel });
      if (t + 1e-9 < nextRev) continue;
      const phase = ((t - 0.09) % 5) / 5;
      const tilt = phase < 0.5 ? -20 + 80 * phase : 60 - 80 * phase;
      const rays = Array.from({ length: 60 }, (_, i) => ({ azimuthDeg: i * 6 - 90, rangeM: 1 }));
      const directions = raysToWorldPoints(rays, tilt);
      const points: Vec3[] = directions.map((direction, i) => {
        const range = castRange(origin, direction) + 0.01 * Math.sin(i * 19 + t * 7);
        return [direction[0] * range, direction[1] * range, direction[2] * range];
      });
      engine.addRevolution(t - 0.09, points);
      const stats = engine.getStats();
      if (stats.batches === 1) seededPoints = stats.mapPoints;
      if (stats.state === 'lost') {
        lost = true;
        if (!recovered) expect(stats.mapPoints).toBe(seededPoints);
      }
      if (lost && stats.state === 'tracking') recovered = true;
      nextRev += 0.18;
    }
    expect(lost).toBe(true);
    expect(recovered).toBe(true);
    expect(engine.getStats().state).toBe('tracking');
  });

  it('recovers a walking, turning trajectory from lidar + IMU', () => {
    const mount = mountRotation('portrait');
    const engine = new MappingEngine({ mount: 'portrait', ...DEFAULT_ENGINE_OPTIONS });
    const speed = 0.3;
    const yawRate = (5 * Math.PI) / 180;
    const truth = (t: number): { R: Mat3; p: Vec3 } => {
      // Stationary for the first 5.5 s while the map is seeded, then walking and turning.
      const m = Math.max(0, t - 5.5);
      const yaw = yawRate * m;
      return { R: expSO3([0, 0, yaw]), p: [2 + speed * m * Math.cos(yaw * 0.5), 2.5 + speed * m * Math.sin(yaw * 0.5), 1.2] };
    };

    const duration = 14;
    const imuDt = 0.02;
    const revDt = 0.1;
    let nextRev = revDt;
    for (let step = 1; step * imuDt <= duration; step += 1) {
      const t = step * imuDt;
      const { R } = truth(t);
      const specific = applyRot(transpose(R), [0, 0, G]);
      engine.addImu({
        t,
        gyroDegS: applyRot(transpose(mount), [0, 0, t > 5.5 ? 5 : 0]),
        accel: applyRot(transpose(mount), specific),
      });
      if (t + 1e-9 >= nextRev) {
        const tilt = 20 * Math.sin((2 * Math.PI * (t - revDt / 2)) / 5);
        const pose = truth(t - revDt / 2);
        const rays = Array.from({ length: 360 }, (_, i) => ({ azimuthDeg: i - 90, rangeM: 1 }));
        // Direction of every ray in the scanner frame; the range is whatever the room says it is.
        const directions = raysToWorldPoints(rays, tilt, false, 90, 0.05, 30);
        const points: Vec3[] = [];
        for (const d of directions) {
          const worldDir = applyRot(pose.R, d);
          const range = castRange(pose.p, worldDir);
          if (Number.isFinite(range) && range < 12) points.push([d[0] * range, d[1] * range, d[2] * range]);
        }
        engine.addRevolution(t - revDt / 2, points);
        nextRev += revDt;
      }
    }

    const stats = engine.getStats();
    expect(stats.batches).toBeGreaterThan(8);
    expect(stats.state).toBe('tracking');
    const last = engine.getTrajectory()[engine.getTrajectory().length - 1];
    const expected = truth(last.t);
    // Map frame starts at the first batch pose, so compare displacement relative to it.
    const first = engine.getTrajectory()[0];
    const start = truth(first.t);
    const travelled = Math.hypot(last.pose.t[0] - first.pose.t[0], last.pose.t[1] - first.pose.t[1]);
    const trueTravelled = Math.hypot(expected.p[0] - start.p[0], expected.p[1] - start.p[1]);
    expect(Math.abs(travelled - trueTravelled)).toBeLessThan(0.25);
    expect(engine.map.pointCount).toBeGreaterThan(500);
  });
});
