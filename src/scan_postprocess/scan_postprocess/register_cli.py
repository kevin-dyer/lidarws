"""register_scans: combine two or more scans taken from different vantage points."""
from __future__ import annotations

import argparse
import sys

from .colorize import FRAMES
from .ply_io import read_cloud, write_cloud
from .registration import RegistrationError, RegistrationParams, register_clouds, voxel_merge


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='register_scans',
        description='Align 2+ point clouds (global then local registration) into one combined cloud.',
    )
    parser.add_argument('plys', nargs='+', help='input point clouds (.ply); the first defines the output frame')
    parser.add_argument('-o', '--output', default='registered.ply', help='output .ply (default: registered.ply)')
    parser.add_argument('--voxel', type=float, help='alignment resolution in metres (default: from scene size)')
    parser.add_argument('--merge-voxel', type=float, help='also thin the combined cloud to this voxel size (m)')
    parser.add_argument('--no-ground', action='store_true', help='skip ground levelling (for scenes with no visible ground)')
    parser.add_argument('--min-overlap', type=float, default=0.20,
                        help='minimum overlap fraction for an accepted alignment (default 0.20)')
    parser.add_argument('--frame', choices=sorted(FRAMES), default='frd',
                        help='scanner axes, used only to show the result upright in RViz (default frd: z down)')
    parser.add_argument('--no-view', action='store_true', help='do not publish to / launch RViz')
    parser.add_argument('--no-rviz', action='store_true', help='publish the ROS topic but do not launch RViz')
    parser.add_argument('--debug', action='store_true',
                        help='show each stage of the alignment live in RViz, one colour per scan')
    parser.add_argument('--debug-delay', type=float, default=0.15, help='seconds between debug updates')
    parser.add_argument('--ascii', action='store_true', help='write an ASCII PLY')
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if len(args.plys) < 2:
        parser.error('at least two .ply files are required')
    if args.debug and args.no_view:
        parser.error('--debug needs the viewer; remove --no-view')

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

        clouds = [read_cloud(path) for path in args.plys]
        params = RegistrationParams(voxel=args.voxel, min_fine_fitness=args.min_overlap,
                                  use_ground=not args.no_ground)
        observer = None
        if args.debug and viewer:
            def observer(stage, items):
                viewer.show_alignment(stage, items, args.debug_delay)

        result = register_clouds(clouds, params, observer)
        for k in result.excluded:
            print(f'warning: cloud {k + 1} could not be aligned (too little shared structure); left out.',
                  file=sys.stderr)
        combined = voxel_merge(result.cloud, args.merge_voxel) if args.merge_voxel else result.cloud
        write_cloud(args.output, combined, ascii_format=args.ascii)

        for pair in result.pairs:
            print(f'cloud {pair.index + 1}: overlap {pair.fine_fitness:.0%}, rms {pair.fine_rmse * 100:.2f} cm')
        colour = 'with' if combined.colors is not None else 'no'
        print(f'Wrote {args.output} ({len(combined)} points, {colour} colour)')
        if viewer:
            viewer.publish(combined.points, combined.colors)
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
