"""process_group: colour every scan that has a photo, then register all scans in a shared folder."""
from __future__ import annotations

import argparse
import os
import re
import sys
from datetime import datetime
from typing import List, Optional

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


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='process_group',
        description='Colour all scans in a shared folder that have a photo, then register them all into one cloud.',
    )
    p.add_argument('folder', help='folder of shared scans (one sub-folder per scan with .ply and optional .jpg)')
    p.add_argument('-o', '--output-dir', help='where results go (default: <folder>/processed)')
    p.add_argument('--hfov', type=float, default=65.0, help='photo horizontal field of view in degrees')
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

    clouds: List[Cloud] = []
    for ply in scans:
        name = os.path.splitext(os.path.basename(ply))[0]
        cloud = read_cloud(ply)
        image_path = find_image(ply)
        if image_path:
            result = colorize_points(cloud.points, load_image(image_path), hfov_deg=args.hfov, frame=args.frame)
            cloud = Cloud(cloud.points, result.colors)
            write_cloud(os.path.join(out_dir, f'{name}_colored.ply'), cloud)
            print(f'{name}: coloured {result.stats["colored"]}/{result.stats["points"]} points')
        else:
            print(f'{name}: no photo, using plain point cloud')
        clouds.append(cloud)

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
        result = register_clouds(clouds, params, observer)
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
