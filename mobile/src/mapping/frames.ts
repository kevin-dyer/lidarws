import type { Point3 } from '../scanner/pointCloud';
import type { Vec3 } from './se3';

/**
 * The app's point-cloud convention is x forward, y right, z down (FRD), while odometry works in
 * x forward, y left, z up (FLU). The flip is its own inverse.
 */
export function flipFrdFlu(p: Vec3): Point3 {
  return [p[0], -p[1], -p[2]];
}
