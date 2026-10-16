"""process_group: colour every scan that has a photo, then register all scans in a shared folder."""
from __future__ import annotations

import argparse
import os
import re
import sys
from datetime import datetime
from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import calibrate_group
from .blend import blend_colors
from .feature_calibration import calibrate_by_features
from .colorize import FRAMES, colorize_points, load_image
from .ply_io import Cloud, read_cloud, write_cloud
from .meshing import build_mesh, write_mesh
from .registration import RegistrationError, RegistrationParams, register_clouds, voxel_merge

IMAGE_EXTS = ('.jpg', '.jpeg', '.png')


def _sort_key(path: str):
    # Shared scan names look like 10_16_2026_9_57_13_AM; fall back to the name if they do not parse.
    stem = os.path.splitext(os.path.basename(path))[0]
    m = re.fullmatch(r'(\d+)_(\d+)_(\d+)_(\d+)_(\d+)_(\d+)_([AP]M)', stem)
    if m:
        mo, d, y, h, mi, s, ap = m.groups()
        try:
            return (0, datetime.strptime(f'{mo}/{d}/{y} {h}:{mi}:{s} {ap}', '%m/%d/%Y %I:%M:%S %p'), stem)
        except ValueError:
            pass
    return (1, datetime.min, stem)


def find_scans(folder: str, exclude_dir: Optional[str] = None) -> List[str]:
    plys = []
    skip = os.path.abspath(exclude_dir) if exclude_dir else None
    for root, _dirs, files in os.walk(folder):
        if os.path.basename(root) == 'processed' or (skip and os.path.abspath(root) == skip):
            continue
        for name in files:
            if name.lower().endswith('.ply') and not name.lower().endswith('_colored.ply'):
                plys.append(os.path.join(root, name))
    return sorted(plys, key=_sort_key)


def find_image(ply: str) -> Optional[str]:
    directory, name = os.path.split(ply)
    stem = os.path.splitext(name)[0]
    for ext in IMAGE_EXTS:
        for candidate in (stem + ext, stem + ext.upper(), stem + '_photo' + ext):
            path = os.path.join(directory, candidate)
            if os.path.exists(path):
                return path
    return None


def _colors_or_gray(cloud: Cloud) -> np.ndarray:
    if cloud.colors is not None:
        return cloud.colors
    return np.tile(np.array([128, 128, 128], dtype=np.uint8), (len(cloud), 1))


def calibrate_cameras(plain, images, names, transforms, args) -> list:
    """Estimates each photo's camera pose from the registered group."""
    cameras = [None] * len(plain)
    if not args.no_calibrate:
        calibration = calibrate_by_features(
            [c.points for c in plain], images, transforms, hfov_deg=args.hfov, frame=args.frame)
        if calibration.ok:
            cameras = calibration.cameras
            print(f'camera calibration (photo features + lidar): rms {calibration.rms_px:.1f} px, '
                  f'consistent matches per photo {calibration.inliers}')
            if calibration.message:
                print(f'  {calibration.message}')
        else:
            print(f'camera calibration from photo features failed ({calibration.message}); '
                  'trying photometric consistency')
            fallback = calibrate_group([c.points for c in plain], images, transforms,
                                       hfov_deg=args.hfov, frame=args.frame)
            cameras = fallback.cameras
            if not fallback.improved:
                print('  not enough shared view to improve on the nominal camera; using it')
        for name, cam in zip(names, cameras):
            if cam is not None:
                angles = Rotation.from_matrix(cam.rotation).as_euler('xyz', degrees=True)
                print(f'  {name}: pitch {angles[0]:.1f}, yaw {angles[1]:.1f}, roll {angles[2]:.1f} deg, '
                      f'fov {cam.fov_deg:.1f} deg, offset {np.round(cam.translation, 2).tolist()} m')
    return cameras


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='process_group',
        description='Colour all scans in a shared folder that have a photo, then register them all into one cloud.',
    )
    p.add_argument('folder', help='folder of shared scans (one sub-folder per scan with .ply and optional .jpg)')
    p.add_argument('-o', '--output-dir', help='where results go (default: <folder>/processed)')
    p.add_argument('--hfov', type=float, default=65.0, help='nominal field of view along the photo\'s long side in degrees (refined by calibration)')
    p.add_argument('--no-calibrate', action='store_true', help='skip camera pose/focal estimation; assume the photo looks along +x')
    p.add_argument('--frame', choices=sorted(FRAMES), default='frd')
    p.add_argument('--voxel', type=float, help='registration resolution in metres')
    p.add_argument('--merge-voxel', type=float, help='thin the combined cloud to this voxel size (m)')
    p.add_argument('--min-overlap', type=float, default=0.20)
    p.add_argument('--no-mesh', action='store_true', help='skip surface reconstruction')
    p.add_argument('--mesh-depth', type=int, default=9, help='Poisson octree depth; higher = more detail (default 9)')
    p.add_argument('--no-view', action='store_true')
    p.add_argument('--no-rviz', action='store_true')
    p.add_argument('--debug', action='store_true', help='show the alignment live in RViz')
    p.add_argument('--debug-delay', type=float, default=0.15)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not os.path.isdir(args.folder):
        print(f'error: {args.folder} is not a folder', file=sys.stderr)
        return 2
    out_dir = args.output_dir or os.path.join(args.folder, 'processed')
    scans = find_scans(args.folder, out_dir)
    if len(scans) < 2:
        print(f'error: need at least two .ply scans in {args.folder}, found {len(scans)}', file=sys.stderr)
        return 2
    os.makedirs(out_dir, exist_ok=True)

    names = [os.path.splitext(os.path.basename(ply))[0] for ply in scans]
    plain = [read_cloud(ply) for ply in scans]
    images = []
    for ply, name in zip(scans, names):
        image_path = find_image(ply)
        images.append(load_image(image_path) if image_path else None)
        if not image_path:
            print(f'{name}: no photo, using plain point cloud')

    viewer = None
    try:
        if not args.no_view:
            try:
                from .rviz_view import RvizViewer
                viewer = RvizViewer(frame=args.frame, launch_rviz=not args.no_rviz)
            except RuntimeError as error:
                if args.debug:
                    raise
                print(f'warning: {error} Continuing without RViz.', file=sys.stderr)
        observer = None
        if args.debug and viewer:
            def observer(stage, items):
                viewer.show_alignment(stage, items, args.debug_delay)

        params = RegistrationParams(voxel=args.voxel, min_fine_fitness=args.min_overlap)
        result = register_clouds(plain, params, observer)
        transforms = [np.asarray(t) for t in result.transformations]
        cameras = calibrate_cameras(plain, images, names, transforms, args)
        if any(im is not None for im in images):
            blended = blend_colors(result.cloud.points, result.source_index, images, cameras, transforms,
                                   frame=args.frame, hfov_deg=args.hfov)
            result.cloud = Cloud(result.cloud.points, blended.colors)
            print(f'blended colour: {blended.stats["colored"]}/{blended.stats["points"]} points coloured, '
                  f'{blended.stats["multi_view"]} seen by several photos '
                  f'(mean disagreement {blended.disagreement:.0f}/255)')
            for i, name in enumerate(names):
                mine = np.asarray(result.source_index) == i
                write_cloud(os.path.join(out_dir, f'{name}_colored.ply'),
                            Cloud(plain[i].points, blended.colors[mine]))
        combined = voxel_merge(result.cloud, args.merge_voxel) if args.merge_voxel else result.cloud
        output = os.path.join(out_dir, 'registered.ply')
        write_cloud(output, combined)
        for pair in result.pairs:
            print(f'scan {pair.index + 1}: overlap {pair.fine_fitness:.0%}, rms {pair.fine_rmse * 100:.2f} cm')
        print(f'Wrote {output} ({len(combined)} points)')
        mesh = None
        if not args.no_mesh:
            origins = [t[:3, 3] for t in result.transformations]
            vox = args.merge_voxel or 0.03
            mesh = build_mesh(combined if args.merge_voxel else result.cloud, result.source_index if not args.merge_voxel else None,
                              origins, voxel=vox, depth=args.mesh_depth)
            mesh_path = os.path.join(out_dir, 'registered_mesh.ply')
            write_mesh(mesh_path, mesh)
            print(f'Wrote {mesh_path} ({len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles)')
        if viewer:
            viewer.publish(combined.points, combined.colors)
            if mesh is not None:
                viewer.publish_mesh(mesh.vertices, mesh.triangles, mesh.colors)
            viewer.hold()
        return 0
    except (RegistrationError, ValueError, IOError, RuntimeError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2
    finally:
        if viewer:
            viewer.close()


if __name__ == '__main__':
    sys.exit(main())
