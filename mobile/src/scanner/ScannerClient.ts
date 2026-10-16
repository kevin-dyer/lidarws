import { Buffer } from 'buffer';
import dgram from 'react-native-udp';
import { raysToWorldPoints, type Point3 } from './pointCloud';
import {
  CHANNEL_CONTROL,
  CHANNEL_LIDAR,
  CHANNEL_TILT,
  decodeFrame,
  encodeFrame,
  MAX_TILT_SPEED_DEG_S,
  RplidarParser,
  STEPS_PER_DEGREE,
} from './protocol';

const BOARD_PORT = 4210;
const HOST_PORT = 4211;
const BROADCAST_ADDRESS = '255.255.255.255';
const FALLBACK_ADDRESS = '192.168.4.1';

export interface ScanSettings {
  startAngle: number;
  stopAngle: number;
  durationSeconds: number;
  homeBeforeScan?: boolean;
  frontHalfOnly?: boolean;
}

export interface ScanProgress {
  phase: 'preparing' | 'scanning';
  percent: number;
  secondsRemaining: number;
  status: string;
}

export interface ScanResult {
  points: Point3[];
  durationSeconds: number;
}

export class ScanCancelledError extends Error {
  constructor() {
    super('Scan cancelled.');
    this.name = 'ScanCancelledError';
  }
}

type LineWaiter = {
  predicate: (line: string) => boolean;
  resolve: (line: string) => void;
  reject: (error: Error) => void;
  timeout: ReturnType<typeof setTimeout>;
};

export class ScannerClient {
  private socket = this.createSocket();
  private bound = false;
  private boardAddress: string | null = null;
  private tiltText = '';
  private currentTiltDeg = 0;
  private captureActive = false;
  private frontHalfOnly = false;
  private scanning = false;
  private lastPongAt = 0;
  private lastLidarRxAt = 0;
  private cancelActiveScan: (() => void) | null = null;
  private lineWaiters: LineWaiter[] = [];
  private readonly lidarParser = new RplidarParser();
  private readonly points: Point3[] = [];
  private onPoints?: (points: Point3[]) => void;
  private lastPointsEmit = 0;
  private onDeviceChange: (address: string | null) => void = () => {};
  private onError: (message: string) => void = () => {};
  private packetsReceived = 0;
  private sendFailures = 0;
  private lastSendError = '';

  constructor(
    onDeviceChange?: (address: string | null) => void,
    onError?: (message: string) => void,
  ) {
    if (onDeviceChange) this.onDeviceChange = onDeviceChange;
    if (onError) this.onError = onError;
  }

  private createSocket(): ReturnType<typeof dgram.createSocket> {
    const socket = dgram.createSocket('udp4');
    socket.on('message', (message, info) => this.receive(message, info.address));
    socket.on('error', (error) => this.onError(`Network error: ${error.message}`));
    return socket;
  }

  // Android pins a socket to the network that was active when it was created,
  // so a fresh socket is required after joining or leaving a Wi-Fi network.
  resetSocket(): void {
    try {
      this.socket.close();
    } catch {
      // Already closed.
    }
    this.socket = this.createSocket();
    this.bound = false;
    this.boardAddress = null;
    this.onDeviceChange(null);
  }

  get deviceAddress(): string | null {
    return this.boardAddress;
  }

  async open(): Promise<void> {
    if (this.bound) return;
    await new Promise<void>((resolve, reject) => {
      const onBind = (): void => {
        this.bound = true;
        try {
          this.socket.setBroadcast(true);
          resolve();
        } catch (error) {
          reject(error instanceof Error ? error : new Error(String(error)));
        }
      };
      this.socket.bind(HOST_PORT, '0.0.0.0', onBind);
    });
  }

  async probe(address: string, timeoutMs = 3000): Promise<boolean> {
    await this.open();
    this.boardAddress = null;
    const interval = setInterval(() => {
      this.sendControl(new Uint8Array([80, 73, 78, 71]), address);
    }, 300);
    try {
      const deadline = Date.now() + timeoutMs;
      while (!this.boardAddress && Date.now() < deadline) {
        await new Promise((resolve) => setTimeout(resolve, 100));
      }
      if (this.boardAddress) this.onDeviceChange(this.boardAddress);
      return this.boardAddress !== null;
    } finally {
      clearInterval(interval);
    }
  }

  /** Verifies the board answers and that the lidar responds to a health request. */
  async checkConnection(timeoutMs = 1500): Promise<'ok' | 'lidar-missing' | 'offline'> {
    if (this.scanning) return 'ok';
    const address = this.boardAddress;
    if (!address) return 'offline';
    const started = Date.now();
    this.sendControl(new Uint8Array([80, 73, 78, 71]), address);
    this.sendLidar(new Uint8Array([0xa5, 0x52]));
    while (Date.now() - started < timeoutMs) {
      await new Promise((resolve) => setTimeout(resolve, 100));
      if (this.lastPongAt >= started && this.lastLidarRxAt >= started) return 'ok';
    }
    if (this.lastPongAt < started) {
      if (this.scanning) return 'ok';
      this.boardAddress = null;
      this.onDeviceChange(null);
      return 'offline';
    }
    return 'lidar-missing';
  }

  async status(timeoutMs = 2500): Promise<{ joined: boolean; ip: string }> {
    const response = this.waitForControl((text) => text.startsWith('STATUS,'), timeoutMs);
    this.sendControl(new Uint8Array([83, 84, 65, 84, 85, 83]));
    const [, joined, ip] = (await response).split(',');
    return { joined: joined === '1', ip };
  }

  async discover(timeoutMs = 4500, extraAddresses: string[] = []): Promise<string> {
    await this.open();
    this.boardAddress = null;
    this.packetsReceived = 0;
    this.sendFailures = 0;
    this.onDeviceChange(null);
    const interval = setInterval(() => {
      this.sendControl(new Uint8Array([80, 73, 78, 71]), BROADCAST_ADDRESS);
      this.sendControl(new Uint8Array([80, 73, 78, 71]), FALLBACK_ADDRESS);
      for (const address of extraAddresses) {
        this.sendControl(new Uint8Array([80, 73, 78, 71]), address);
      }
    }, 350);
    try {
      const deadline = Date.now() + timeoutMs;
      while (!this.boardAddress && Date.now() < deadline) {
        await new Promise((resolve) => setTimeout(resolve, 100));
      }
      if (!this.boardAddress) {
        const detail = this.sendFailures > 0
          ? ` Send failed: ${this.lastSendError}.`
          : ` Packets received: ${this.packetsReceived}.`;
        throw new Error(`Scanner not found. Join its Wi-Fi or the same home network, then retry.${detail}`);
      }
      return this.boardAddress;
    } finally {
      clearInterval(interval);
    }
  }

  async provisionHomeWifi(ssid: string, password: string): Promise<void> {
    if (!this.boardAddress) await this.discover();
    const name = Buffer.from(ssid, 'utf8');
    const secret = Buffer.from(password, 'utf8');
    if (
      ssid.trim().length === 0 ||
      name.length > 32 ||
      (secret.length > 0 && secret.length < 8) ||
      secret.length > 63
    ) {
      throw new Error('SSID must be 1–32 bytes; password must be blank or 8–63 bytes.');
    }
    const payload = new Uint8Array(6 + name.length + secret.length);
    payload.set([87, 73, 70, 73, name.length, secret.length]);
    payload.set(name, 6);
    payload.set(secret, 6 + name.length);
    const responsePromise = this.waitForControl((text) => text.startsWith('WIFI_'), 5000);
    this.sendControl(payload);
    const response = await responsePromise;
    if (response !== 'WIFI_OK') {
      throw new Error('The scanner could not save the home Wi-Fi settings.');
    }
  }

  async startScan(
    settings: ScanSettings,
    update: (progress: ScanProgress) => void,
    onPoints?: (points: Point3[]) => void,
  ): Promise<ScanResult> {
    if (!this.boardAddress) await this.discover();
    if (this.scanning) throw new Error('A scan is already in progress.');
    if (Math.abs(settings.startAngle) > 45 || Math.abs(settings.stopAngle) > 45 ||
        settings.startAngle >= settings.stopAngle) {
      throw new Error('Choose a start angle below the stop angle, within ±45°.');
    }

    this.scanning = true;
    this.frontHalfOnly = settings.frontHalfOnly === true;
    this.points.length = 0;
    this.onPoints = onPoints;
    this.lastPointsEmit = 0;
    this.currentTiltDeg = 0;
    const span = settings.stopAngle - settings.startAngle;
    const actualDuration = Math.max(settings.durationSeconds, span / MAX_TILT_SPEED_DEG_S);
    const speed = span / actualDuration;
    const returnSeconds = Math.abs(settings.stopAngle) / MAX_TILT_SPEED_DEG_S;
    const expectedDuration = actualDuration + returnSeconds;
    let sweepStartedAt = 0;
    let timer: ReturnType<typeof setInterval> | undefined;
    const cancellable = <T,>(work: Promise<T>): Promise<T> =>
      Promise.race([
        work,
        new Promise<never>((_, reject) => {
          this.cancelActiveScan = () => reject(new ScanCancelledError());
        }),
      ]);

    const progress = (status: string, complete = false): void => {
      const elapsed = (Date.now() - sweepStartedAt) / 1000;
      const percent = complete ? 100 : Math.min(99, Math.floor((elapsed / expectedDuration) * 100));
      update({
        phase: 'scanning',
        percent,
        secondsRemaining: complete ? 0 : Math.max(0, Math.ceil(expectedDuration - elapsed)),
        status,
      });
    };

    try {
      const homing = settings.homeBeforeScan !== false;
      update({
        phase: 'preparing',
        percent: 0,
        secondsRemaining: Math.ceil(expectedDuration),
        status: homing ? 'Homing tilt stage…' : 'Moving to the start angle…',
      });
      this.lidarParser.reset();
      this.sendLidar(new Uint8Array([0xa5, 0x20]));
      if (homing) {
        const home = this.waitForTiltLine(
          (line) => line.includes('Homing complete') || line.startsWith('Error homing'),
          90000,
        );
        this.sendTilt('A\n');
        const homeLine = await cancellable(home);
        if (homeLine.startsWith('Error homing')) {
          throw new Error('The tilt stage could not find its home position.');
        }
      }

      const speedResponse = this.waitForTiltLine(
        (line) => line.startsWith('Max tilt speed:') || line.startsWith('ERR SPEED'),
        3000,
      );
      this.sendTilt(`S${speed.toFixed(3)}\n`);
      if ((await speedResponse).startsWith('ERR SPEED')) {
        throw new Error('The scanner rejected the requested tilt speed.');
      }

      // The first sweep sample is emitted once the stage has reached the start angle.
      const sweepStart = this.waitForTiltLine((line) => /^\d+,-?\d+$/.test(line), 90000);
      this.sendTilt(`C${settings.startAngle.toFixed(2)},${settings.stopAngle.toFixed(2)}\n`);
      await cancellable(sweepStart);
      sweepStartedAt = Date.now();
      this.lidarParser.beginCapture();
      this.captureActive = true;
      progress('Scanning the scene…');
      timer = setInterval(() => progress('Scanning the scene…'), 250);
      const sweepEnd = this.waitForTiltLine((line) => line === 'X', expectedDuration * 1000 + 60000);
      await cancellable(sweepEnd);
      this.captureActive = false;
      this.sendLidar(new Uint8Array([0xa5, 0x25]));
      if (this.points.length === 0) {
        throw new Error('No LiDAR points were received. Check the sensor connection and try again.');
      }
      progress('Scan complete', true);
      return { points: [...this.points], durationSeconds: actualDuration };
    } catch (error) {
      this.captureActive = false;
      this.sendLidar(new Uint8Array([0xa5, 0x25]));
      this.sendTilt('X\n');
      if (error instanceof ScanCancelledError) throw error;
      throw error instanceof Error ? error : new Error(String(error));
    } finally {
      this.scanning = false;
      this.onPoints = undefined;
      this.cancelActiveScan = null;
      for (const waiter of this.lineWaiters) {
        clearTimeout(waiter.timeout);
        waiter.reject(new ScanCancelledError());
      }
      this.lineWaiters = [];
      if (timer) clearInterval(timer);
    }
  }

  cancelScan(): void {
    if (!this.scanning) return;
    this.captureActive = false;
    this.sendLidar(new Uint8Array([0xa5, 0x25]));
    this.sendTilt('X\n');
    this.cancelActiveScan?.();
  }

  close(): void {
    this.cancelScan();
    this.socket.close();
    this.bound = false;
  }

  private receive(packet: Uint8Array, sender: string): void {
    this.packetsReceived += 1;
    const frame = decodeFrame(packet);
    if (!frame) return;
    if (frame.channel === CHANNEL_CONTROL) {
      const text = Buffer.from(frame.payload).toString('utf8');
      if (text === 'PONG') {
        this.lastPongAt = Date.now();
        this.boardAddress = sender;
        this.onDeviceChange(sender);
      } else {
        this.resolveControlWaiters(text);
      }
      return;
    }
    if (sender !== this.boardAddress) return;
    if (frame.channel === CHANNEL_TILT) {
      this.receiveTilt(frame.payload);
    } else if (frame.channel === CHANNEL_LIDAR) {
      this.lastLidarRxAt = Date.now();
      if (!this.scanning) return;
      this.lidarParser.push(frame.payload, (rays) => {
        if (this.captureActive) {
          this.points.push(...raysToWorldPoints(rays, this.currentTiltDeg, this.frontHalfOnly));
          const now = Date.now();
          if (this.onPoints && now - this.lastPointsEmit >= 300) {
            this.lastPointsEmit = now;
            this.onPoints([...this.points]);
          }
        }
      });
    }
  }

  private receiveTilt(payload: Uint8Array): void {
    this.tiltText += Buffer.from(payload).toString('utf8');
    let newline = this.tiltText.indexOf('\n');
    while (newline >= 0) {
      const line = this.tiltText.slice(0, newline).replace(/\r/g, '').trim();
      this.tiltText = this.tiltText.slice(newline + 1);
      if (line) this.handleTiltLine(line);
      newline = this.tiltText.indexOf('\n');
    }
  }

  private handleTiltLine(line: string): void {
    if (/^\d+,\-?\d+$/.test(line)) {
      const steps = Number(line.split(',')[1]);
      this.currentTiltDeg = steps / STEPS_PER_DEGREE;
    }
    const waiterIndex = this.lineWaiters.findIndex((waiter) => waiter.predicate(line));
    if (waiterIndex >= 0) {
      const [waiter] = this.lineWaiters.splice(waiterIndex, 1);
      clearTimeout(waiter.timeout);
      waiter.resolve(line);
    }
  }

  private waitForTiltLine(predicate: (line: string) => boolean, timeoutMs: number): Promise<string> {
    return new Promise((resolve, reject) => {
      const waiter: LineWaiter = {
        predicate,
        resolve,
        reject,
        timeout: setTimeout(() => {
          this.lineWaiters = this.lineWaiters.filter((candidate) => candidate !== waiter);
          reject(new Error('Timed out waiting for the tilt stage.'));
        }, timeoutMs),
      };
      this.lineWaiters.push(waiter);
    });
  }

  private waitForControl(predicate: (text: string) => boolean, timeoutMs: number): Promise<string> {
    return new Promise((resolve, reject) => {
      const waiter: LineWaiter = {
        predicate,
        resolve,
        reject,
        timeout: setTimeout(() => {
          this.lineWaiters = this.lineWaiters.filter((candidate) => candidate !== waiter);
          reject(new Error('Timed out waiting for a scanner response.'));
        }, timeoutMs),
      };
      this.lineWaiters.push(waiter);
    });
  }

  private resolveControlWaiters(text: string): void {
    const waiterIndex = this.lineWaiters.findIndex((waiter) => waiter.predicate(text));
    if (waiterIndex < 0) return;
    const [waiter] = this.lineWaiters.splice(waiterIndex, 1);
    clearTimeout(waiter.timeout);
    waiter.resolve(text);
  }

  private sendControl(payload: Uint8Array, address = this.boardAddress): void {
    if (!address || !this.bound) return;
    this.sendFrame(CHANNEL_CONTROL, payload, address);
  }

  private sendLidar(payload: Uint8Array): void {
    if (!this.boardAddress) return;
    this.sendFrame(CHANNEL_LIDAR, payload, this.boardAddress);
  }

  private sendTilt(text: string): void {
    if (!this.boardAddress) return;
    this.sendFrame(CHANNEL_TILT, Buffer.from(text, 'utf8'), this.boardAddress);
  }

  private sendFrame(channel: number, payload: Uint8Array, address: string): void {
    const frame = encodeFrame(channel, payload);
    this.socket.send(Buffer.from(frame), 0, frame.length, BOARD_PORT, address, (error) => {
      if (error) {
        this.sendFailures += 1;
        this.lastSendError = error.message;
      }
    });
  }
}
