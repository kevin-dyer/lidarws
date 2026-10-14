import { decimate, projectPoints } from './projection';

describe('projection', () => {
  it('decimates to at most max points', () => {
    const pts = Array.from({ length: 1000 }, (_, i) => [i, 0, 0] as [number, number, number]);
    expect(decimate(pts, 100)).toHaveLength(100);
    expect(decimate(pts, 5000)).toHaveLength(1000);
  });

  it('projects forward points to screen centre at zero yaw/pitch', () => {
    const out = projectPoints([[2, 0, 0], [2, 0, 1]], 0, 0, 2);
    expect(out[0].x).toBeCloseTo(0);
    expect(out[1].y).toBeCloseTo(-0.5);
    expect(out[1].band).toBeLessThan(out[0].band);
  });
});

import { encodeAsciiPly, parseAsciiPly } from './pointCloud';

describe('ply', () => {
  it('round-trips and decimates', () => {
    const pts = Array.from({ length: 100 }, (_, i) => [i, 1, 2] as [number, number, number]);
    const text = encodeAsciiPly(pts);
    expect(parseAsciiPly(text)).toHaveLength(100);
    expect(parseAsciiPly(text, 10).length).toBeLessThanOrEqual(10);
  });
});
