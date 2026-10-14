import { encodeAsciiPly, raysToWorldPoints } from './pointCloud';
import { decodeFrame, encodeFrame, CHANNEL_TILT, RplidarParser } from './protocol';

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

  it('uses the current ROS pipeline azimuth offset and tilt rotation', () => {
    const [point] = raysToWorldPoints([{ azimuthDeg: 270, rangeM: 2 }], 30);
    expect(point[0]).toBeCloseTo(Math.sqrt(3));
    expect(point[1]).toBeCloseTo(0);
    expect(point[2]).toBeCloseTo(1);
  });

  it('writes a standard ASCII PLY vertex list', () => {
    const ply = encodeAsciiPly([[1, 2, 3]]);
    expect(ply).toContain('format ascii 1.0');
    expect(ply).toContain('element vertex 1');
    expect(ply.endsWith('1.000000 2.000000 3.000000\n')).toBe(true);
  });
});
