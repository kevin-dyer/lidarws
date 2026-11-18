import { DEFAULT_ICP, registerScan, type IcpOptions } from './icp';
import { ImuOrientation, type ImuSample, type PhoneMount } from './inertial';
import {
  applyRot,
  logSO3,
  mulMat,
  norm,
  transformPoint,
  transpose,
  type Mat3,
  type Pose,
  type Vec3,
} from './se3';
import { VoxelMap, voxelDownsample } from './voxelMap';

export type TrackingState = 'initializing' | 'tracking' | 'lost';

export interface TrajectoryPoint {
  t: number;
  pose: Pose;
}

export interface MappingEngineOptions {
  mount: PhoneMount;
  /** Lidar revolutions are pooled for this long before each scan-to-map match. */
  batchSeconds: number;
  /** The first batch seeds the map and should span a full tilt sweep; the scanner should be held still for it. */
  initSeconds: number;
  /** Voxel size used to thin each batch before matching. */
  scanVoxelM: number;
  icp: IcpOptions;
}

export const DEFAULT_ENGINE_OPTIONS: Omit<MappingEngineOptions, 'mount'> = {
  batchSeconds: 0.5,
  initSeconds: 5,
  scanVoxelM: 0.1,
  icp: DEFAULT_ICP,
};

const MIN_MATCHES = 120;
const MAX_RMSE_M = 0.08;
const MAX_ROTATION_CORRECTION_RAD = (20 * Math.PI) / 180;
const MAX_TRANSLATION_CORRECTION_M = 1.0;
const MAX_SPEED_M_S = 3;

interface PendingRevolution {
  t: number;
  points: Vec3[];
}

export interface EngineStats {
  state: TrackingState;
  batches: number;
  lostBatches: number;
  mapPoints: number;
  lastMatches: number;
  lastRmse: number;
  distanceM: number;
}

/**
 * LiDAR-inertial odometry: gyro/accelerometer give a gravity-aligned orientation prior and deskewing,
 * and each batch of lidar revolutions is registered point-to-plane against the growing voxel map.
 * The map frame has z up and starts at the pose where mapping began.
 */
export class MappingEngine {
  readonly map = new VoxelMap();
  private readonly imu: ImuOrientation;
  private pending: PendingRevolution[] = [];
  private pose: Pose | null = null;
  private poseImuR: Mat3 | null = null;
  private poseT = 0;
  private velocity: Vec3 = [0, 0, 0];
  private state: TrackingState = 'initializing';
  private batches = 0;
  private lostBatches = 0;
  private lastMatches = 0;
  private lastRmse = 0;
  private batchWindowSeconds: number;
  private distanceM = 0;
  private readonly trajectory: TrajectoryPoint[] = [];

  constructor(private readonly options: MappingEngineOptions) {
    this.imu = new ImuOrientation(options.mount);
    this.batchWindowSeconds = options.batchSeconds;
  }

  addImu(sample: ImuSample): void {
    this.imu.update(sample);
  }

  get currentPose(): Pose | null {
    return this.pose;
  }

  getTrajectory(): readonly TrajectoryPoint[] {
    return this.trajectory;
  }

  getStats(): EngineStats {
    return {
      state: this.state,
      batches: this.batches,
      lostBatches: this.lostBatches,
      mapPoints: this.map.pointCount,
      lastMatches: this.lastMatches,
      lastRmse: this.lastRmse,
      distanceM: this.distanceM,
    };
  }

  /** `points` are in the scanner frame (tilt already applied); `t` is the revolution's mid time in seconds. */
  addRevolution(t: number, points: Vec3[]): void {
    if (!this.imu.ready || points.length === 0) return;
    this.pending.push({ t, points });
    const first = this.pending[0].t;
    const window = this.pose === null ? this.options.initSeconds : this.batchWindowSeconds;
    if (t - first >= window) {
      const batch = this.pending;
      this.pending = [];
      this.processBatch(batch);
    }
  }

  private processBatch(batch: PendingRevolution[]): void {
    const tRef = batch[batch.length - 1].t;
    const imuRef = this.imu.orientationAt(tRef);
    if (!imuRef) return;

    const first = this.pose === null || this.poseImuR === null;
    // Rotation delta from the IMU alone, so slow IMU yaw drift never leaks into the map frame.
    const predicted: Pose = first
      ? { R: imuRef, t: [0, 0, 0] }
      : this.predict(imuRef, tRef);

    const scan = this.deskew(batch, imuRef, predicted, tRef);
    if (scan.length === 0) return;

    let result: Pose = predicted;
    let accepted = first;
    if (!first) {
      const icp = registerScan(scan, this.map, predicted, predicted, this.options.icp);
      this.lastMatches = icp.matches;
      this.lastRmse = icp.rmse;
      const rotationJump = norm(logSO3(mulMat(icp.pose.R, transpose(predicted.R))));
      const translationJump = Math.hypot(
        icp.pose.t[0] - predicted.t[0],
        icp.pose.t[1] - predicted.t[1],
        icp.pose.t[2] - predicted.t[2],
      );
      accepted =
        icp.matches >= MIN_MATCHES &&
        icp.rmse <= MAX_RMSE_M &&
        rotationJump <= MAX_ROTATION_CORRECTION_RAD &&
        translationJump <= MAX_TRANSLATION_CORRECTION_M;
      if (accepted) result = icp.pose;
    }

    this.batches += 1;
    const previous = this.pose;
    const dt = tRef - this.poseT;
    if (accepted) {
      this.state = 'tracking';
      if (previous && dt > 0) this.updateVelocity(previous, result, dt);
      this.map.insert(scan.map((p) => transformPoint(result, p)));
      this.trajectory.push({ t: tRef, pose: result });
    } else {
      this.state = 'lost';
      this.lostBatches += 1;
      this.velocity = [0, 0, 0];
      if (this.lastMatches < MIN_MATCHES) {
        this.batchWindowSeconds = Math.min(
          Math.max(3, this.options.batchSeconds),
          this.batchWindowSeconds * 4,
        );
      }
    }
    if (previous && accepted) {
      this.distanceM += Math.hypot(
        result.t[0] - previous.t[0],
        result.t[1] - previous.t[1],
        result.t[2] - previous.t[2],
      );
    }
    // Keep following the inertial estimate even when matching failed, so tracking can recover.
    this.pose = result;
    this.poseImuR = imuRef;
    this.poseT = tRef;
  }

  private predict(imuRef: Mat3, tRef: number): Pose {
    const previous = this.pose as Pose;
    const delta = mulMat(transpose(this.poseImuR as Mat3), imuRef);
    const dt = tRef - this.poseT;
    return {
      R: mulMat(previous.R, delta),
      t: [
        previous.t[0] + this.velocity[0] * dt,
        previous.t[1] + this.velocity[1] * dt,
        previous.t[2] + this.velocity[2] * dt,
      ],
    };
  }

  private updateVelocity(from: Pose, to: Pose, dt: number): void {
    const next: Vec3 = [
      (to.t[0] - from.t[0]) / dt,
      (to.t[1] - from.t[1]) / dt,
      (to.t[2] - from.t[2]) / dt,
    ];
    const speed = norm(next);
    const limit = speed > MAX_SPEED_M_S ? MAX_SPEED_M_S / speed : 1;
    this.velocity = [
      0.5 * this.velocity[0] + 0.5 * next[0] * limit,
      0.5 * this.velocity[1] + 0.5 * next[1] * limit,
      0.5 * this.velocity[2] + 0.5 * next[2] * limit,
    ];
  }

  /** Expresses every revolution in the scanner frame at `tRef`, compensating rotation and constant-velocity motion. */
  private deskew(batch: PendingRevolution[], imuRef: Mat3, reference: Pose, tRef: number): Vec3[] {
    const imuRefT = transpose(imuRef);
    const refRt = transpose(reference.R);
    const out: Vec3[] = [];
    for (const rev of batch) {
      const imuAt = this.imu.orientationAt(rev.t);
      if (!imuAt) continue;
      const rotation = mulMat(imuRefT, imuAt);
      const shift = applyRot(refRt, [
        this.velocity[0] * (rev.t - tRef),
        this.velocity[1] * (rev.t - tRef),
        this.velocity[2] * (rev.t - tRef),
      ]);
      for (const p of rev.points) {
        const r = applyRot(rotation, p);
        out.push([r[0] + shift[0], r[1] + shift[1], r[2] + shift[2]]);
      }
    }
    return voxelDownsample(out, this.options.scanVoxelM);
  }
}
