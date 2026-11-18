import { Accelerometer, DeviceMotion, Gyroscope } from 'expo-sensors';
import { Buffer } from 'buffer';
import { Platform } from 'react-native';
type Subscription = { remove: () => void };
import type { ScannerClient, MappingSession } from '../scanner/ScannerClient';
import type { Point3 } from '../scanner/pointCloud';
import { flipFrdFlu } from './frames';
import { DEFAULT_ENGINE_OPTIONS, MappingEngine, type EngineStats } from './odometry';
import type { MappingSettings } from './settings';
import { DesktopStream } from './desktopStream';
import { encodeImage, encodeImu, encodeTimedLidar } from './desktopProtocol';
import { deviceMotionGyro, mountRotation } from './inertial';
import { applyRot, type Vec3 } from './se3';

const IMU_INTERVAL_MS = 20;
const UI_INTERVAL_MS = 700;
const LIVE_POINT_LIMIT = 20000;

export interface MappingSnapshot {
  stats: EngineStats;
  /** Live map in the app's x-forward, y-right, z-down convention, thinned for display. */
  points: Point3[];
  elapsedSeconds: number;
  /** Seconds of the initial still period left; 0 once the first map has been built. */
  holdStillSeconds: number;
}

export interface MappingOutput {
  points: Point3[];
  durationSeconds: number;
  trajectoryJson: string;
}

export interface MappingRunHooks {
  onSnapshot: (snapshot: MappingSnapshot) => void;
  onError: (error: Error) => void;
  /** Starts recording forward video and resolves with the file URI after `stopVideo`. */
  startVideo: () => Promise<string | undefined>;
  stopVideo: () => void;
  desktopHost?: string;
  captureFrame?: () => Promise<string>;
}

/**
 * One hand-held mapping session: wires the phone's gyro/accelerometer and the camera recording to the
 * scanner's lidar stream and keeps a live point cloud via the LiDAR-inertial odometry engine.
 */
export class MappingRun {
  private readonly engine: MappingEngine;
  private session: MappingSession | null = null;
  private motion: Subscription | null = null;
  private timer: ReturnType<typeof setInterval> | undefined;
  private video: Promise<string | undefined> | null = null;
  private startedAt = 0;
  private videoStartedAt = 0;
  private lastBatches = -1;
  private livePoints: Point3[] = [];
  private stopped = false;
  private desktop: DesktopStream | null = null;
  private desktopSensors: Subscription[] = [];
  private imuWatchdog: ReturnType<typeof setInterval> | undefined;

  constructor(
    private readonly scanner: ScannerClient,
    private readonly settings: MappingSettings,
    private readonly hooks: MappingRunHooks,
  ) {
    this.engine = new MappingEngine({
      mount: settings.phoneMount,
      ...DEFAULT_ENGINE_OPTIONS,
      initSeconds: settings.sweepPeriodSeconds,
    });
  }

  async start(): Promise<void> {
    if (!(await DeviceMotion.isAvailableAsync())) {
      throw new Error('This device has no motion sensors, which mobile mapping needs.');
    }
    if (this.hooks.desktopHost) {
      if (!this.hooks.captureFrame) throw new Error('Desktop mapping needs a camera frame source.');
      if (!(await Accelerometer.isAvailableAsync()) || !(await Gyroscope.isAvailableAsync())) {
        throw new Error('Desktop mapping needs both an accelerometer and a gyroscope.');
      }
      this.desktop = new DesktopStream(this.hooks.desktopHost, (error) => this.failDesktop(error));
      try {
        await this.desktop.open();
      } catch (error) {
        this.desktop.close();
        this.desktop = null;
        throw error;
      }
      this.startDesktopImu();
    }
    DeviceMotion.setUpdateInterval(IMU_INTERVAL_MS);
    this.motion = DeviceMotion.addListener((data) => {
      const gravity = data.accelerationIncludingGravity;
      const rate = data.rotationRate;
      if (!gravity || !rate) return;
      this.engine.addImu({
        t: Date.now() / 1000,
        gyroDegS: deviceMotionGyro(rate, Platform.OS),
        accel: [gravity.x, gravity.y, gravity.z],
      });
    });

    try {
      this.session = await this.scanner.startMapping(this.settings, {
        onRevolution: ({ points, tStart, tEnd }) => {
          this.engine.addRevolution((tStart + tEnd) / 2, points.map(flipFrdFlu));
        },
        onTimedRevolution: this.desktop
          ? (points) => {
              try {
                void this.desktop?.send(encodeTimedLidar(points), 'lidar').catch((error: unknown) => this.failDesktop(error));
              } catch (error) {
                this.failDesktop(error);
              }
            }
          : undefined,
        onError: (error) => this.hooks.onError(error),
      });
    } catch (error) {
      this.dispose();
      throw error;
    }

    if (this.stopped) {
      // Aborted while the stage was still homing.
      await this.session.stop().catch(() => {});
      return;
    }
    this.startedAt = Date.now();
    this.videoStartedAt = Date.now() / 1000;
    if (this.desktop) {
      void this.streamFrames();
    } else {
      this.video = this.hooks.startVideo().catch(() => undefined);
    }
    this.timer = setInterval(() => this.publish(), UI_INTERVAL_MS);
  }

  private failDesktop(error: unknown): void {
    if (!this.stopped) this.hooks.onError(error instanceof Error ? error : new Error(String(error)));
  }

  private startDesktopImu(): void {
    let acceleration: Vec3 | null = null;
    let accelerationTime = 0;
    let epochOffset: number | null = null;
    let lastValid = Date.now();
    const rotation = mountRotation(this.settings.phoneMount);
    Accelerometer.setUpdateInterval(IMU_INTERVAL_MS);
    Gyroscope.setUpdateInterval(IMU_INTERVAL_MS);
    this.desktopSensors.push(Accelerometer.addListener((sample) => {
      acceleration = applyRot(rotation, [sample.x * 9.80665, sample.y * 9.80665, sample.z * 9.80665]);
      accelerationTime = sample.timestamp;
    }));
    this.desktopSensors.push(Gyroscope.addListener((sample) => {
      if (!acceleration || Math.abs(sample.timestamp - accelerationTime) > 0.1) return;
      lastValid = Date.now();
      if (epochOffset === null) epochOffset = Date.now() / 1000 - sample.timestamp;
      const angular = applyRot(rotation, [sample.x, sample.y, sample.z]);
      const message = encodeImu(sample.timestamp + epochOffset, acceleration, angular);
      void this.desktop?.send(message, 'imu').catch((error: unknown) => this.failDesktop(error));
    }));
    this.imuWatchdog = setInterval(() => {
      if (Date.now() - lastValid > 3000) this.failDesktop(new Error('Phone IMU data stopped or is not synchronized.'));
    }, 1000);
  }

  private async streamFrames(): Promise<void> {
    const capture = this.hooks.captureFrame;
    const stream = this.desktop;
    if (!capture || !stream) return;
    try {
      while (!this.stopped) {
        const timestamp = Date.now() / 1000;
        const jpeg = await capture();
        if (this.stopped) break;
        await stream.send(encodeImage(timestamp, Buffer.from(jpeg, 'base64')), 'image');
      }
    } catch (error) {
      this.failDesktop(error);
    }
  }

  private publish(): void {
    const stats = this.engine.getStats();
    if (stats.batches !== this.lastBatches) {
      this.lastBatches = stats.batches;
      this.livePoints = this.engine.map.getPoints(LIVE_POINT_LIMIT).map(flipFrdFlu);
    }
    const elapsedSeconds = (Date.now() - this.startedAt) / 1000;
    this.hooks.onSnapshot({
      stats,
      points: this.livePoints,
      elapsedSeconds,
      holdStillSeconds:
        stats.state === 'initializing' ? Math.max(0, Math.ceil(this.settings.sweepPeriodSeconds - elapsedSeconds)) : 0,
    });
  }

  private dispose(): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = undefined;
    this.motion?.remove();
    this.motion = null;
    this.desktopSensors.forEach((sensor) => sensor.remove());
    this.desktopSensors = [];
    if (this.imuWatchdog) clearInterval(this.imuWatchdog);
    this.imuWatchdog = undefined;
    this.desktop?.close();
    this.desktop = null;
  }

  /** Stops the stage, lidar, sensors and recording, and returns the finished map. */
  async stop(): Promise<MappingOutput & { videoUri?: string }> {
    if (this.stopped) throw new Error('Mapping has already been stopped.');
    this.stopped = true;
    this.dispose();
    const durationSeconds = Math.round((Date.now() - this.startedAt) / 1000);
    this.hooks.stopVideo();
    await this.session?.stop();
    const videoUri = this.video ? await this.video : undefined;

    const points = this.engine.map.getPoints().map(flipFrdFlu);
    const trajectory = this.engine.getTrajectory().map(({ t, pose }) => ({
      t,
      position: pose.t,
      rotation: pose.R,
    }));
    const trajectoryJson = JSON.stringify({
      version: 1,
      frame: 'FLU: x forward, y left, z up; origin at the sensor pose when mapping began. Saved points use x, -y, -z.',
      rotation: 'row-major 3x3, scanner-to-map',
      videoStartEpochS: this.video ? this.videoStartedAt : null,
      settings: this.settings,
      poses: trajectory,
    });
    return { points, durationSeconds, trajectoryJson, videoUri };
  }

  /** Aborts without producing output (e.g. the screen was left). */
  async abort(): Promise<void> {
    if (this.stopped) return;
    this.stopped = true;
    this.dispose();
    this.hooks.stopVideo();
    await this.session?.stop().catch(() => {});
  }
}
