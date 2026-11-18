import type { PhoneMount } from './inertial';

/** Fastest tilt speed the firmware allows during continuous oscillation (matches HOMING_SPEED_DEG_S). */
export const MAX_OSCILLATION_SPEED_DEG_S = 40;
export const MAX_TILT_ANGLE_DEG = 45;

export interface MappingSettings {
  /** Seconds for one full up-and-down cycle of the tilt stage. */
  sweepPeriodSeconds: number;
  minAngleDeg: number;
  maxAngleDeg: number;
  phoneMount: PhoneMount;
  maxRangeM: number;
}

export const DEFAULT_MAPPING_SETTINGS: MappingSettings = {
  sweepPeriodSeconds: 5,
  minAngleDeg: -20,
  maxAngleDeg: 20,
  phoneMount: 'portrait',
  maxRangeM: 12,
};

/** Shortest period the tilt stage can follow for a given angular span. */
export function minimumSweepPeriod(minAngleDeg: number, maxAngleDeg: number): number {
  return (2 * (maxAngleDeg - minAngleDeg)) / MAX_OSCILLATION_SPEED_DEG_S;
}

/** Returns a user-facing problem description, or null when the settings are usable. */
export function validateMappingSettings(settings: MappingSettings): string | null {
  const { sweepPeriodSeconds: period, minAngleDeg: min, maxAngleDeg: max, maxRangeM } = settings;
  if (![period, min, max, maxRangeM].every(Number.isFinite)) return 'Enter a number in every field.';
  if (min >= max) return 'The lowest tilt angle must be below the highest.';
  if (Math.abs(min) > MAX_TILT_ANGLE_DEG || Math.abs(max) > MAX_TILT_ANGLE_DEG) {
    return `Tilt angles must stay within ±${MAX_TILT_ANGLE_DEG}°.`;
  }
  const shortest = minimumSweepPeriod(min, max);
  if (period < shortest) {
    return `A ${max - min}° sweep needs at least ${shortest.toFixed(1)} s per cycle; increase the period or narrow the angles.`;
  }
  if (period > 60) return 'The sweep period can be at most 60 seconds.';
  if (maxRangeM < 1 || maxRangeM > 30) return 'Maximum range must be between 1 and 30 m.';
  return null;
}
