import { Buffer } from 'buffer';
import dgram from 'react-native-udp';
import { fragmentMessage } from './desktopProtocol';

const PORT = 5005;
const MAX_PENDING_LIDAR_SCANS = 8;

interface PendingLidarScan {
  message: Uint8Array;
  resolve: () => void;
  reject: (error: unknown) => void;
}

export class DesktopStream {
  private readonly socket = dgram.createSocket('udp4');
  private closed = false;
  private imagePending = false;
  private readonly lidarQueue: PendingLidarScan[] = [];
  private lidarSending = false;
  private lastAcknowledgement = 0;
  private heartbeat: ReturnType<typeof setInterval> | undefined;

  constructor(private readonly host: string, private readonly onError: (error: Error) => void) {
    this.socket.on('error', onError);
  }

  async open(): Promise<void> {
    await new Promise<void>((resolve, reject) => {
      this.socket.bind(0, '0.0.0.0', (error) => {
        if (error) reject(error);
        else resolve();
      });
    });
    await new Promise<void>((resolve, reject) => {
      const timeout = setTimeout(() => reject(new Error('Desktop bridge did not respond on UDP port 5005.')), 4000);
      this.socket.on('message', (message, info) => {
        if (info.address === this.host && info.port === PORT && Buffer.from(message).toString('utf8') === 'LIVO_OK') {
          this.lastAcknowledgement = Date.now();
          clearTimeout(timeout);
          resolve();
        }
      });
      this.sendPacket(Buffer.from('LIVO_HELLO')).catch((error: unknown) => {
        clearTimeout(timeout);
        reject(error);
      });
    });
    this.heartbeat = setInterval(() => {
      if (Date.now() - this.lastAcknowledgement > 9000) {
        if (this.heartbeat) clearInterval(this.heartbeat);
        this.heartbeat = undefined;
        this.onError(new Error('Lost contact with the desktop bridge.'));
        return;
      }
      void this.sendPacket(Buffer.from('LIVO_HELLO')).catch((error: unknown) => {
        if (!this.closed) this.onError(error instanceof Error ? error : new Error(String(error)));
      });
    }, 3000);
  }

  private sendPacket(packet: Uint8Array): Promise<void> {
    if (this.closed) return Promise.reject(new Error('Desktop stream is closed.'));
    return new Promise((resolve, reject) => {
      this.socket.send(packet, 0, packet.length, PORT, this.host, (error) => {
        if (error) reject(error);
        else resolve();
      });
    });
  }

  async send(message: Uint8Array, kind: 'imu' | 'image' | 'lidar'): Promise<void> {
    if (kind === 'lidar') return this.enqueueLidar(message);
    if (kind === 'image' && this.imagePending) {
      throw new Error('The desktop image stream cannot keep up.');
    }
    if (kind === 'image') this.imagePending = true;
    try {
      await this.sendMessage(message);
    } finally {
      if (kind === 'image') this.imagePending = false;
    }
  }

  private enqueueLidar(message: Uint8Array): Promise<void> {
    if (this.closed) return Promise.reject(new Error('Desktop stream is closed.'));
    if (this.lidarQueue.length + Number(this.lidarSending) >= MAX_PENDING_LIDAR_SCANS) {
      return Promise.reject(new Error('The desktop LiDAR queue is full; the stream cannot keep up.'));
    }
    return new Promise((resolve, reject) => {
      this.lidarQueue.push({ message, resolve, reject });
      if (!this.lidarSending) void this.drainLidar();
    });
  }

  private async drainLidar(): Promise<void> {
    this.lidarSending = true;
    try {
      while (this.lidarQueue.length > 0) {
        const scan = this.lidarQueue.shift();
        if (!scan) break;
        try {
          await this.sendMessage(scan.message);
          scan.resolve();
        } catch (error) {
          scan.reject(error);
          this.rejectQueuedLidar(error);
          break;
        }
      }
    } finally {
      this.lidarSending = false;
    }
  }

  private rejectQueuedLidar(error: unknown): void {
    for (const scan of this.lidarQueue.splice(0)) scan.reject(error);
  }

  private async sendMessage(message: Uint8Array): Promise<void> {
    const random = (): number => Math.floor(Math.random() * 0x100000000);
    const fragments = fragmentMessage(message, random(), random());
    for (let index = 0; index < fragments.length; index += 1) {
      await this.sendPacket(fragments[index]);
      if (index + 1 < fragments.length) await new Promise((resolve) => setTimeout(resolve, 1));
    }
  }

  close(): void {
    if (this.closed) return;
    this.closed = true;
    this.rejectQueuedLidar(new Error('Desktop stream is closed.'));
    if (this.heartbeat) clearInterval(this.heartbeat);
    this.heartbeat = undefined;
    this.socket.close();
  }
}
