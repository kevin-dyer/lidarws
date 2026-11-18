/** Recent (time, tilt angle) samples from the stage, used to give each lidar ray its own tilt while the stage moves. */
export class TiltTrack {
  private readonly samples: { t: number; deg: number }[] = [];

  constructor(private readonly keepSeconds = 3) {}

  clear(): void {
    this.samples.length = 0;
  }

  push(t: number, deg: number): void {
    const last = this.samples[this.samples.length - 1];
    if (last && t <= last.t) return;
    this.samples.push({ t, deg });
    while (this.samples.length > 2 && this.samples[0].t < t - this.keepSeconds) this.samples.shift();
  }

  get latest(): number | null {
    const last = this.samples[this.samples.length - 1];
    return last ? last.deg : null;
  }

  /** Linear interpolation between samples; extrapolates up to 0.1 s past the newest sample. */
  angleAt(t: number): number | null {
    const s = this.samples;
    if (s.length === 0) return null;
    if (s.length === 1) return s[0].deg;
    if (t <= s[0].t) return s[0].deg;
    const last = s[s.length - 1];
    if (t >= last.t) {
      const prev = s[s.length - 2];
      const rate = (last.deg - prev.deg) / (last.t - prev.t);
      return last.deg + rate * Math.min(t - last.t, 0.1);
    }
    let lo = 0;
    let hi = s.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (s[mid].t <= t) lo = mid;
      else hi = mid;
    }
    const f = (t - s[lo].t) / (s[hi].t - s[lo].t);
    return s[lo].deg + (s[hi].deg - s[lo].deg) * f;
  }
}
