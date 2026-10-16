"""Camera-to-scanner calibration from photo feature matches.

SIFT features are matched between the photos of overlapping scans (photo-to-photo matching is
reliable, unlike matching a photo against a rendering of a depth-only point cloud). Each match is a
real scene point seen from two viewpoints: the lidar gives it a 3D position through one photo's ray,
and the other photo must then see that 3D point at the matched pixel. The camera rotations, offsets
and a shared focal length are solved by minimising that reprojection error (a robust PnP over all
photos), re-casting the rays as the poses improve.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .calibration import AXES, CameraModel, nominal_focal

FEATURE_SIDE = 1600
MAX_FEATURES = 6000
RATIO = 0.8
CELL = 8  # pixels per depth-buffer cell when casting rays
DEPTH_SPREAD = 0.08  # relative depth variation allowed around a ray hit (rejects silhouettes)
INLIER_PX = 20.0  # at full photo resolution
CAP_PX = 80.0
PRIOR_ROT = np.radians(20.0)
PRIOR_OFFSET = 0.3
PRIOR_LOG_FOCAL = 0.35
MIN_INLIERS = 25
ROUNDS = 6


@dataclass
class FeatureCalibration:
    cameras: List[Optional[CameraModel]]
    inliers: List[int]
    rms_px: float
    ok: bool
    message: str = ''


def _features(image: np.ndarray):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    scale = min(1.0, FEATURE_SIDE / max(gray.shape))
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else gray
    small = cv2.createCLAHE(2.0, (8, 8)).apply(small)
    kp, desc = cv2.SIFT_create(MAX_FEATURES).detectAndCompute(small, None)
    pts = np.float32([k.pt for k in kp]) / scale if kp else np.zeros((0, 2), np.float32)
    return pts, desc


def match_photos(images: Sequence[Optional[np.ndarray]]):
    """Verified pixel matches for every photo pair: {(i, j): (pts_i, pts_j)}."""
    feats = [_features(im) if im is not None else None for im in images]
    matcher = cv2.BFMatcher()
    pairs = {}
    for i in range(len(images)):
        for j in range(i + 1, len(images)):
            if feats[i] is None or feats[j] is None or feats[i][1] is None or feats[j][1] is None:
                continue
            raw = matcher.knnMatch(feats[i][1], feats[j][1], k=2)
            good = [m[0] for m in raw if len(m) == 2 and m[0].distance < RATIO * m[1].distance]
            if len(good) < 15:
                continue
            a = feats[i][0][[m.queryIdx for m in good]]
            b = feats[j][0][[m.trainIdx for m in good]]
            _, mask = cv2.findFundamentalMat(a, b, cv2.USAC_MAGSAC, 3.0, 0.999, 10000)
            if mask is None:
                continue
            keep = mask.ravel().astype(bool)
            if keep.sum() >= 15:
                pairs[(i, j)] = (a[keep].astype(float), b[keep].astype(float))
    return pairs


class _Camera:
    """Pinhole camera parameterised by a rotation vector and offset relative to the nominal pose."""

    def __init__(self, width, height, frame):
        self.w, self.h, self.axes = width, height, AXES[frame]

    def project(self, x, rotvec, offset, focal):
        cam = ((x - offset) @ self.axes.T) @ Rotation.from_rotvec(rotvec).as_matrix().T
        z = np.where(cam[:, 2] > 1e-3, cam[:, 2], 1e-3)
        return np.stack([self.w / 2 + focal * cam[:, 0] / z, self.h / 2 + focal * cam[:, 1] / z], 1), cam[:, 2]

    def cast(self, cloud, px, rotvec, offset, focal):
        """3D scan-frame hits for pixels `px`, using a min-depth buffer of `cloud`; NaN if unreliable."""
        uv, depth = self.project(cloud, rotvec, offset, focal)
        gw, gh = int(self.w // CELL) + 1, int(self.h // CELL) + 1
        ok = (depth > 0.3) & (uv[:, 0] >= 0) & (uv[:, 0] < self.w) & (uv[:, 1] >= 0) & (uv[:, 1] < self.h)
        cell = (uv[ok, 1] // CELL).astype(int) * gw + (uv[ok, 0] // CELL).astype(int)
        buf = np.full(gw * gh, np.inf)
        np.minimum.at(buf, cell, depth[ok])
        buf = buf.reshape(gh, gw)
        # fill small gaps and measure the local spread so silhouettes are rejected
        lo = -cv2.dilate(-np.where(np.isfinite(buf), buf, 1e6).astype(np.float32), np.ones((5, 5)))
        hi = cv2.dilate(np.where(np.isfinite(buf), buf, 0).astype(np.float32), np.ones((5, 5)))
        cx, cy = (px[:, 0] // CELL).astype(int), (px[:, 1] // CELL).astype(int)
        inside = (cx >= 0) & (cx < gw) & (cy >= 0) & (cy < gh)
        cx, cy = np.clip(cx, 0, gw - 1), np.clip(cy, 0, gh - 1)
        d = lo[cy, cx]
        good = inside & (d < 1e5) & ((hi[cy, cx] - lo[cy, cx]) < DEPTH_SPREAD * d)
        ray = np.stack([(px[:, 0] - self.w / 2) / focal, (px[:, 1] - self.h / 2) / focal, np.ones(len(px))], 1)
        hit_cam = ray * d[:, None]
        rot = Rotation.from_rotvec(rotvec).as_matrix()
        hit = (hit_cam @ rot) @ self.axes + offset
        hit[~good] = np.nan
        return hit


def _align(u, v):
    """Rotation taking unit vector u onto unit vector v."""
    axis = np.cross(u, v)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.eye(3) if u @ v > 0 else Rotation.from_rotvec(np.pi * _any_perp(u)).as_matrix()
    return Rotation.from_rotvec(axis / n * np.arctan2(n, u @ v)).as_matrix()


def _any_perp(u):
    p = np.cross(u, [1.0, 0, 0] if abs(u[0]) < 0.9 else [0, 1.0, 0])
    return p / np.linalg.norm(p)


class _Group:
    """Photos, matches and the registered lidar geometry shared by all the solving steps."""

    def __init__(self, points, images, transforms, frame, pairs, max_cloud=60000):
        self.images, self.T, self.pairs, self.frame = images, [np.asarray(t) for t in transforms], pairs, frame
        self.inv = [np.linalg.inv(t) for t in self.T]
        world = np.vstack([p @ t[:3, :3].T + t[:3, 3] for p, t in zip(points, self.T)])
        if len(world) > max_cloud:
            world = world[np.random.default_rng(0).choice(len(world), max_cloud, replace=False)]
        self.local = {i: world @ self.inv[i][:3, :3].T + self.inv[i][:3, 3] for i in range(len(images))}
        self.cams = {i: _Camera(im.shape[1], im.shape[0], frame) for i, im in enumerate(images) if im is not None}

    def origin(self, i):
        return self.T[i][:3, 3]

    def to_world(self, i, x):
        return x @ self.T[i][:3, :3].T + self.T[i][:3, 3]

    def to_local(self, i, x):
        return x @ self.inv[i][:3, :3].T + self.inv[i][:3, 3]

    def hits(self, src, px, rot, off, focal):
        """World-frame lidar hits of the rays through pixels `px` of photo `src` (NaN where unreliable)."""
        h = self.cams[src].cast(self.local[src], px, Rotation.from_matrix(rot).as_rotvec(), off, focal)
        return self.to_world(src, h)

    def reproject(self, x, dst, rot, off, focal):
        uv, depth = self.cams[dst].project(self.to_local(dst, x), Rotation.from_matrix(rot).as_rotvec(), off, focal)
        return uv, depth

    def pair_errors(self, poses, focal, only=None):
        """Reprojection error (px) of every match in both directions, inf where there is no lidar hit."""
        out = {}
        for (i, j), (pi, pj) in self.pairs.items():
            if i not in poses or j not in poses or (only and (i, j) != only):
                continue
            for src, dst, ps, pd in ((i, j, pi, pj), (j, i, pj, pi)):
                rs, os_ = poses[src]
                rd, od = poses[dst]
                x = self.hits(src, ps, rs, os_, focal)
                ok = np.isfinite(x).all(axis=1)
                e = np.full(len(ps), np.inf)
                if ok.any():
                    uv, depth = self.reproject(x[ok], dst, rd, od, focal)
                    err = np.linalg.norm(uv - pd[ok], axis=1)
                    err[depth <= 0.1] = np.inf
                    e[ok] = err
                out[(src, dst)] = e
        return out


def _robust(errors):
    return sum(np.minimum(e, CAP_PX).sum() / CAP_PX for e in errors.values())


def _seed_pair(group, pair, focal_nominal, hfovs):
    """Camera poses of two photos from their relative geometry plus the known scan baseline.

    The essential matrix gives the relative camera rotation and the direction between the two
    cameras; the registration gives the same baseline in scanner axes. That fixes all but a roll
    about the baseline, which is then found by checking lidar consistency."""
    i, j = pair
    pi, pj = group.pairs[pair]
    A = AXES[group.frame]
    zero = np.zeros(3)
    best = (np.inf, None)
    d = group.origin(i) - group.origin(j)
    for hfov in hfovs:
        focal = nominal_focal(group.images[i].shape[1], group.images[i].shape[0], hfov)
        K = np.array([[focal, 0, group.images[i].shape[1] / 2], [0, focal, group.images[i].shape[0] / 2], [0, 0, 1]])
        E, _ = cv2.findEssentialMat(pi, pj, K, cv2.RANSAC, 0.999, 3.0)
        if E is None or E.shape != (3, 3):
            continue
        _, R, t, _ = cv2.recoverPose(E, pi, pj, K)
        t = t.ravel()
        Rsi, Rsj = group.T[i][:3, :3], group.T[j][:3, :3]
        u = A @ Rsi.T @ d
        u /= np.linalg.norm(u)
        c = R.T @ t
        c /= np.linalg.norm(c)
        for sign in (1.0, -1.0):
            base = _align(u, sign * c)
            for theta in np.radians(np.arange(0, 360, 10.0)):
                rc_i = Rotation.from_rotvec(c * theta).as_matrix() @ base
                rc_j = R @ rc_i @ A @ Rsi.T @ Rsj @ A.T
                poses = {i: (rc_i, zero), j: (rc_j, zero)}
                cost = _robust(group.pair_errors(poses, focal, only=pair))
                if cost < best[0]:
                    best = (cost, (rc_i, rc_j, focal))
    return best


def calibrate_by_features(points, images, transforms, hfov_deg=65.0, frame='frd', pairs=None) -> FeatureCalibration:
    from scipy.optimize import minimize
    n = len(images)
    cameras: List[Optional[CameraModel]] = [None] * n
    pairs = match_photos(images) if pairs is None else pairs
    if not pairs:
        return FeatureCalibration(cameras, [0] * n, 0.0, False, 'no photo features could be matched between scans')
    group = _Group(points, images, transforms, frame, pairs)
    focal_nominal = nominal_focal(images[min(group.cams)].shape[1], images[min(group.cams)].shape[0], hfov_deg)
    seed = max(pairs, key=lambda k: len(pairs[k][0]))
    cost, found = _seed_pair(group, seed, focal_nominal, np.arange(40.0, 95.0, 5.0))
    if found is None:
        return FeatureCalibration(cameras, [0] * n, 0.0, False, 'could not recover the relative camera geometry')
    rc_i, rc_j, focal = found
    solved = {seed[0]: rc_i, seed[1]: rc_j}
    zero = np.zeros(3)

    def unpack(p):
        ids = sorted(solved)
        return ({i: (Rotation.from_rotvec(p[3 * k:3 * k + 3]).as_matrix(), zero) for k, i in enumerate(ids)},
                focal_nominal * np.exp(p[-1]), ids)

    def objective(p):
        poses, f, _ = unpack(p)
        return _robust(group.pair_errors(poses, f)) + 0.02 * (p[-1] / PRIOR_LOG_FOCAL) ** 2

    ids = sorted(solved)
    p0 = np.concatenate([np.concatenate([Rotation.from_matrix(solved[i]).as_rotvec() for i in ids]),
                         [np.log(focal / focal_nominal)]])
    res = minimize(objective, p0, method='Powell', options={'xtol': 1e-4, 'ftol': 1e-5, 'maxfev': 1500})
    poses, focal, _ = unpack(res.x)
    solved = {i: poses[i][0] for i in poses}

    # remaining photos: PnP against lidar points hit by the already solved cameras
    changed = True
    while changed:
        changed = False
        for (i, j), (pi, pj) in pairs.items():
            for src, dst, ps, pd in ((i, j, pi, pj), (j, i, pj, pi)):
                if src in solved and dst not in solved:
                    x = group.hits(src, ps, solved[src], zero, focal)
                    ok = np.isfinite(x).all(axis=1)
                    if ok.sum() < 8:
                        continue
                    xl = group.to_local(dst, x[ok]) @ AXES[frame].T
                    w, h = group.cams[dst].w, group.cams[dst].h
                    K = np.array([[focal, 0, w / 2], [0, focal, h / 2], [0, 0, 1]])
                    good, rvec, _tvec, inl = cv2.solvePnPRansac(
                        xl, pd[ok], K, None, flags=cv2.SOLVEPNP_EPNP, reprojectionError=INLIER_PX, iterationsCount=500)
                    if good and inl is not None and len(inl) >= MIN_INLIERS:
                        solved[dst] = Rotation.from_rotvec(rvec.ravel()).as_matrix()
                        changed = True

    poses = {i: (solved[i], zero) for i in solved}
    errors = group.pair_errors(poses, focal)
    all_e = np.concatenate(list(errors.values()))
    inl_mask = all_e < INLIER_PX
    inliers = [0] * n
    for (src, dst), e in errors.items():
        inliers[dst] += int((e < INLIER_PX).sum())
    rms = float(np.sqrt(np.mean(all_e[inl_mask] ** 2))) if inl_mask.any() else 0.0
    if inl_mask.sum() < MIN_INLIERS * len(errors) or inl_mask.mean() < 0.3:
        return FeatureCalibration(cameras, inliers, rms, False,
                                  f'only {int(inl_mask.sum())}/{len(all_e)} photo matches agree with the lidar')
    for i, rot in solved.items():
        cam = CameraModel(focal, images[i].shape[1], images[i].shape[0], frame=frame)
        cam.rotation = rot
        cameras[i] = cam
    notes = []
    for i in range(n):
        if images[i] is not None and cameras[i] is None:
            cam, corr = _fit_to_colours(points, images, transforms, cameras, i, focal, frame)
            cameras[i] = cam
            notes.append(f'photo {i}: no feature links to other photos; '
                         + (f'rotation fitted to the other scans\' colours (correlation {corr:.2f})'
                            if corr else 'left at the nominal pose'))
    return FeatureCalibration(cameras, inliers, rms, True, '; '.join(notes))


def _fit_to_colours(points, images, transforms, cameras, k, focal, frame):
    """Rotation of a photo that shares no features with the others: the already calibrated photos
    colour the other scans, and this photo's rotation is searched to agree with those colours."""
    from scipy.optimize import minimize
    import scipy.ndimage as ndi
    from .colorize import colorize_points

    inv = np.linalg.inv(transforms[k])
    pts, cols = [], []
    for i, cam in enumerate(cameras):
        if cam is None or i == k:
            continue
        res = colorize_points(points[i], images[i], camera=cam, frame=frame)
        world = points[i][res.confident] @ np.asarray(transforms[i])[:3, :3].T + np.asarray(transforms[i])[:3, 3]
        pts.append(world @ inv[:3, :3].T + inv[:3, 3])
        cols.append(res.colors[res.confident].astype(float).mean(axis=1))
    nominal = CameraModel(focal, images[k].shape[1], images[k].shape[0], frame=frame)
    if not pts:
        return nominal, 0.0
    pts, cols = np.vstack(pts), np.concatenate(cols)
    pick = np.random.default_rng(0).choice(len(pts), min(len(pts), 20000), replace=False)
    pts, cols = pts[pick], cols[pick]
    h, w = images[k].shape[:2]
    gray = cv2.GaussianBlur(cv2.cvtColor(images[k], cv2.COLOR_RGB2GRAY), (0, 0), 3).astype(float)

    def camera_for(p):
        cam = CameraModel(focal, w, h, frame=frame)
        cam.rotation = Rotation.from_rotvec(p).as_matrix()
        return cam

    def corr(p):
        u, v, depth = camera_for(p).project(pts)
        ok = (depth > 0.3) & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
        if ok.sum() < 2000:
            return 0.0
        g = ndi.map_coordinates(gray, [v[ok], u[ok]], order=1)
        return float(np.corrcoef(g, cols[ok])[0, 1])

    def cost(p):
        return -corr(p) + 0.02 * (p @ p) / np.radians(10.0) ** 2

    rng = np.random.default_rng(1)
    starts = sorted((cost(p), tuple(p)) for p in rng.normal(0, np.radians(12.0), (300, 3)))[:3]
    best = min((minimize(cost, np.array(p), method='Powell', options={'xtol': 1e-3, 'ftol': 1e-4, 'maxfev': 800})
                for _, p in starts), key=lambda r: r.fun)
    gain = corr(best.x) - corr(np.zeros(3))
    if corr(best.x) < 0.3 or gain < 0.15:
        return nominal, 0.0
    return camera_for(best.x), corr(best.x)
