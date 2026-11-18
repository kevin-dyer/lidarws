import { encodeAsciiPly, raysToWorldPoints } from './pointCloud';
import { decodeFrame, encodeFrame, CHANNEL_TILT, RplidarParser } from './protocol';
import { BoardClock } from './boardClock';
import { TimedPacketOrder } from './timedPacketOrder';

function measurementNode(sync: boolean, angleDeg: number, distanceMm: number): number[] {
  const angleField = (Math.round(angleDeg * 64) << 1) | 1;
  const distanceField = Math.round(distanceMm * 4);
  const flags = sync ? 0x01 : 0x02;
  return [
    flags,
    angleField & 0xff,
    (angleField >> 8) & 0xff,
    distanceField & 0xff,
    (distanceField >> 8) & 0xff,
  ];
}

describe('UDP mux framing', () => {
  it('round-trips a framed tilt payload and rejects a corrupted checksum', () => {
    const encoded = encodeFrame(CHANNEL_TILT, new Uint8Array([84, 10]));
    expect(decodeFrame(encoded)).toEqual({
      channel: CHANNEL_TILT,
      payload: new Uint8Array([84, 10]),
    });

    encoded[5] ^= 1;
    expect(decodeFrame(encoded)).toBeNull();
  });
});

describe('ESP32 clock', () => {
  it('handles counter wrap and estimates individual UART byte times', () => {
    const clock = new BoardClock(0xfffffff0, 1000);
    expect(clock.toEpochSeconds(0x10)).toBeCloseTo(1000.000032, 8);
    const times = clock.byteTimes(0x10, 3);
    expect(times[2]).toBeCloseTo(1000.000032, 8);
    expect((times[2] - times[0]) * 1_000_000).toBeCloseTo(2 * 10_000_000 / 115200, 3);
  });

  describe('timed UDP packet ordering', () => {
    beforeEach(() => jest.useFakeTimers());
    afterEach(() => jest.useRealTimers());

    it('delivers reordered packets in sequence, including across counter wrap', () => {
      const delivered: number[] = [];
      const onError = jest.fn();
      const order = new TimedPacketOrder<number>((value) => delivered.push(value), onError);
      order.push(0xfffffffe, 1);
      order.push(0, 3);
      expect(delivered).toEqual([1]);
      order.push(0xffffffff, 2);
      order.push(1, 4);
      order.push(0xffffffff, 2);
      jest.advanceTimersByTime(200);
      expect(delivered).toEqual([1, 2, 3, 4]);
      expect(onError).not.toHaveBeenCalled();
      order.close();
    });

    it('reports a persistent gap without delivering data after the missing packet', () => {
      const delivered: number[] = [];
      const onError = jest.fn();
      const order = new TimedPacketOrder<number>((value) => delivered.push(value), onError);
      order.push(10, 10);
      order.push(12, 12);
      jest.advanceTimersByTime(100);
      expect(delivered).toEqual([10]);
      expect(onError).toHaveBeenCalledWith(expect.objectContaining({ message: expect.stringContaining('11') }));
      order.push(11, 11);
      expect(delivered).toEqual([10]);
      order.close();
    });

    it('cancels a pending gap timeout on close', () => {
      const onError = jest.fn();
      const order = new TimedPacketOrder<number>(() => {}, onError);
      order.push(10, 10);
      order.push(12, 12);
      order.close();
      jest.advanceTimersByTime(200);
      expect(onError).not.toHaveBeenCalled();
    });

    it('fails instead of buffering an unbounded sequence gap', () => {
      const onError = jest.fn();
      const order = new TimedPacketOrder<number>(() => {}, onError);
      order.push(0, 0);
      order.push(33, 33);
      expect(onError).toHaveBeenCalledWith(expect.objectContaining({
        message: expect.stringContaining('packet 1'),
      }));
      order.close();
    });

    it('starts a new timeout when a gap is filled but a later packet is still missing', () => {
      const onError = jest.fn();
      const order = new TimedPacketOrder<number>(() => {}, onError);
      order.push(10, 10);
      order.push(12, 12);
      jest.advanceTimersByTime(80);
      order.push(14, 14);
      order.push(11, 11);
      jest.advanceTimersByTime(20);
      expect(onError).not.toHaveBeenCalled();
      jest.advanceTimersByTime(80);
      expect(onError).toHaveBeenCalledWith(expect.objectContaining({
        message: expect.stringContaining('packet 13'),
      }));
      order.close();
    });

    it('preserves measurement alignment and timestamps when packets arrive out of order', () => {
      const parser = new RplidarParser();
      const revolutions: Array<Array<{ azimuthDeg: number; rangeM: number; time?: number }>> = [];
      const order = new TimedPacketOrder<{ bytes: Uint8Array; times: number[] }>(
        ({ bytes, times }) => parser.push(bytes, (rays) => revolutions.push(rays), times),
        jest.fn(),
      );
      const bytes = [
        0xa5, 0x5a, 5, 0, 0, 0x40, 0x81,
        ...measurementNode(true, 0, 1000),
        ...measurementNode(false, 90, 500),
        ...measurementNode(true, 180, 750),
      ];
      const times = bytes.map((_, i) => i / 1000);
      const part = (start: number, end: number) => ({
        bytes: new Uint8Array(bytes.slice(start, end)),
        times: times.slice(start, end),
      });
      order.push(1, part(0, 9));
      order.push(3, part(14, bytes.length));
      order.push(2, part(9, 14));
      expect(revolutions[0].map((ray) => ray.rangeM)).toEqual([1, 0.5]);
      expect(revolutions[0][0].time).toBeCloseTo(0.007 - 10 / 115200, 9);
      expect(revolutions[0][1].time).toBeCloseTo(0.012 - 10 / 115200, 9);
      order.close();
    });
  });

  it('does not accumulate rounding error across many clock updates', () => {
    const clock = new BoardClock(0, 1_700_000_000);
    for (let i = 1; i <= 100_000; i += 1) clock.toEpochSeconds(i * 1000);
    expect(clock.toEpochSeconds(100_001_000)).toBeCloseTo(1_700_000_100.001, 6);
  });
});

describe('RPLIDAR and point-cloud conversion', () => {
  it('emits one revolution from standard five-byte scan nodes', () => {
    const parser = new RplidarParser();
    const descriptor = [0xa5, 0x5a, 5, 0, 0, 0x40, 0x81];
    const bytes = new Uint8Array([
      ...descriptor,
      ...measurementNode(true, 0, 1000),
      ...measurementNode(false, 90, 500),
      ...measurementNode(true, 180, 750),
    ]);
    const revolutions: Array<Array<{ azimuthDeg: number; rangeM: number }>> = [];

    parser.push(bytes, (rays) => revolutions.push(rays));

    expect(revolutions).toHaveLength(1);
    expect(revolutions[0]).toHaveLength(2);
    expect(revolutions[0][0]).toEqual({ azimuthDeg: 0, rangeM: 1 });
    expect(revolutions[0][1]).toEqual({ azimuthDeg: 90, rangeM: 0.5 });
  });

  it('keeps descriptor state but discards pre-capture rays', () => {
    const parser = new RplidarParser();
    const descriptor = [0xa5, 0x5a, 5, 0, 0, 0x40, 0x81];
    let captureActive = false;
    const revolutions: Array<Array<{ azimuthDeg: number; rangeM: number }>> = [];
    const collectWhenCapturing = (rays: Array<{ azimuthDeg: number; rangeM: number }>): void => {
      if (captureActive) revolutions.push(rays);
    };
    parser.push(
      new Uint8Array([
        ...descriptor,
        ...measurementNode(true, 0, 1000),
        ...measurementNode(true, 90, 1000),
      ]),
      collectWhenCapturing,
    );
    parser.beginCapture();
    captureActive = true;

    parser.push(
      new Uint8Array([
        ...measurementNode(false, 180, 500),
        ...measurementNode(true, 270, 500),
      ]),
      collectWhenCapturing,
    );

    expect(revolutions).toHaveLength(1);
    expect(revolutions[0]).toEqual([{ azimuthDeg: 180, rangeM: 0.5 }]);
  });

  it('keeps the timestamp of each complete measurement across packet boundaries', () => {
    const parser = new RplidarParser();
    const bytes = [
      0xa5, 0x5a, 5, 0, 0, 0x40, 0x81,
      ...measurementNode(true, 0, 1000),
      ...measurementNode(false, 90, 500),
      ...measurementNode(true, 180, 750),
    ];
    const times = bytes.map((_, index) => index / 1000);
    const revolutions: Array<Array<{ azimuthDeg: number; rangeM: number; time?: number }>> = [];
    parser.push(new Uint8Array(bytes.slice(0, 14)), (rays) => revolutions.push(rays), times.slice(0, 14));
    parser.push(new Uint8Array(bytes.slice(14)), (rays) => revolutions.push(rays), times.slice(14));
    expect(revolutions[0][0].time).toBeCloseTo(0.007 - 10 / 115200, 9);
    expect(revolutions[0][1].time).toBeCloseTo(0.012 - 10 / 115200, 9);
    expect(revolutions[0].map((ray) => ray.rangeM)).toEqual([1, 0.5]);
  });

  it('uses the current ROS pipeline azimuth offset and tilt rotation', () => {
    const [point] = raysToWorldPoints([{ azimuthDeg: 270, rangeM: 2 }], 30);
    expect(point[0]).toBeCloseTo(Math.sqrt(3));
    expect(point[1]).toBeCloseTo(0);
    expect(point[2]).toBeCloseTo(1);
  });

  it('crops by range and by a horizontal sweep centred on the front', () => {
    // azimuth 270 + 90 offset = 0 deg from forward; 0 and 180 are straight ahead and behind
    const rays = [
      { azimuthDeg: 270, rangeM: 2 }, // ahead
      { azimuthDeg: 270 + 60, rangeM: 2 }, // 60 deg to one side
      { azimuthDeg: 270 + 180, rangeM: 2 }, // behind
      { azimuthDeg: 270, rangeM: 6 }, // ahead but far
    ];
    expect(raysToWorldPoints(rays, 0)).toHaveLength(4);
    expect(raysToWorldPoints(rays, 0, true)).toHaveLength(3);
    expect(raysToWorldPoints(rays, 0, false, 90, 0.05, 30, 90)).toHaveLength(2);
    expect(raysToWorldPoints(rays, 0, false, 90, 0.05, 30, 360)).toHaveLength(4);
    expect(raysToWorldPoints(rays, 0, false, 90, 0.05, 5)).toHaveLength(3);
    expect(raysToWorldPoints(rays, 0, false, 90, 3)).toHaveLength(1);
  });

  it('writes a standard ASCII PLY vertex list', () => {
    const ply = encodeAsciiPly([[1, 2, 3]]);
    expect(ply).toContain('format ascii 1.0');
    expect(ply).toContain('element vertex 1');
    expect(ply.endsWith('1.000000 2.000000 3.000000\n')).toBe(true);
  });
});
