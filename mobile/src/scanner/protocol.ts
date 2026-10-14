export const CHANNEL_LIDAR = 1;
export const CHANNEL_TILT = 2;
export const CHANNEL_CONTROL = 0x7f;
export const MAX_PAYLOAD = 1190;
export const MAX_TILT_SPEED_DEG_S = 10;
export const STEPS_PER_DEGREE = (200 * 16 * 7.6875) / 360;

export interface MuxFrame {
  channel: number;
  payload: Uint8Array;
}

function crc16(bytes: Uint8Array): number {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 0x8000) !== 0 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

export function encodeFrame(channel: number, payload: Uint8Array): Uint8Array {
  if (payload.length > MAX_PAYLOAD) {
    throw new RangeError(`UDP frame payload exceeds ${MAX_PAYLOAD} bytes`);
  }
  const frame = new Uint8Array(payload.length + 7);
  frame.set([0xa5, 0x5a, channel, payload.length & 0xff, payload.length >> 8]);
  frame.set(payload, 5);
  const checksum = crc16(frame.subarray(2, frame.length - 2));
  frame[frame.length - 2] = checksum & 0xff;
  frame[frame.length - 1] = checksum >> 8;
  return frame;
}

export function decodeFrame(packet: Uint8Array): MuxFrame | null {
  if (packet.length < 7 || packet[0] !== 0xa5 || packet[1] !== 0x5a) {
    return null;
  }
  const length = packet[3] | (packet[4] << 8);
  if (length > MAX_PAYLOAD || packet.length !== length + 7) {
    return null;
  }
  const received = packet[length + 5] | (packet[length + 6] << 8);
  if (crc16(packet.subarray(2, length + 5)) !== received) {
    return null;
  }
  return { channel: packet[2], payload: packet.slice(5, length + 5) };
}

export interface Ray {
  azimuthDeg: number;
  rangeM: number;
}

export class RplidarParser {
  private pending = new Uint8Array(0);
  private measurementMode = false;
  private revolution: Ray[] = [];

  reset(): void {
    this.pending = new Uint8Array(0);
    this.measurementMode = false;
    this.revolution = [];
  }

  beginCapture(): void {
    this.pending = new Uint8Array(0);
    this.revolution = [];
  }

  push(chunk: Uint8Array, onRevolution: (rays: Ray[]) => void): void {
    const joined = new Uint8Array(this.pending.length + chunk.length);
    joined.set(this.pending);
    joined.set(chunk, this.pending.length);
    this.pending = joined;

    if (!this.measurementMode) {
      let descriptorIndex = -1;
      for (let index = 0; index <= this.pending.length - 7; index += 1) {
        if (this.pending[index] === 0xa5 && this.pending[index + 1] === 0x5a) {
          descriptorIndex = index;
          break;
        }
      }
      if (descriptorIndex < 0) {
        this.pending = this.pending.slice(Math.max(0, this.pending.length - 6));
        return;
      }
      if (this.pending.length < descriptorIndex + 7) {
        this.pending = this.pending.slice(descriptorIndex);
        return;
      }
      const descriptorType = this.pending[descriptorIndex + 6];
      this.pending = this.pending.slice(descriptorIndex + 7);
      this.measurementMode = descriptorType === 0x81;
      if (!this.measurementMode) {
        return;
      }
    }

    let offset = 0;
    while (offset + 5 <= this.pending.length) {
      const flags = this.pending[offset];
      const angleField = this.pending[offset + 1] | (this.pending[offset + 2] << 8);
      const distanceField = this.pending[offset + 3] | (this.pending[offset + 4] << 8);
      offset += 5;

      const sync = (flags & 1) !== 0;
      const syncInverse = (flags & 2) !== 0;
      if (sync === syncInverse || (angleField & 1) === 0) {
        continue;
      }

      if (sync && this.revolution.length > 0) {
        onRevolution(this.revolution);
        this.revolution = [];
      }
      const rangeM = distanceField / 4000;
      if (rangeM > 0) {
        this.revolution.push({
          azimuthDeg: (angleField >> 1) / 64,
          rangeM,
        });
      }
    }
    this.pending = this.pending.slice(offset);
  }
}
