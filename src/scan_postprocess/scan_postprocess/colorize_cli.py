"""colorize_scan: add RGB values from a reference photo to a scan PLY."""
from __future__ import annotations

import argparse
import os
import sys

from .colorize import FRAMES, colorize_points, load_image
from .ply_io import Cloud, read_cloud, write_cloud


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='colorize_scan',
        description='Colour a scan using the photo taken at the scanner position (looking along +x).',
    )
    parser.add_argument('ply', help='input point cloud (.ply)')
    parser.add_argument('image', help='reference photo (.jpg/.png)')
    parser.add_argument('-o', '--output', help='output .ply (default: <input>_colored.ply)')
    parser.add_argument('--hfov', type=float, default=65.0,
                        help='horizontal field of view of the photo in degrees (default 65, a typical phone main camera)')
    parser.add_argument('--frame', choices=sorted(FRAMES), default='frd',
                        help='scanner axes: frd = x forward, y right, z down (default); flu = x forward, y left, z up')
    parser.add_argument('--max-range', type=float, default=30.0, help='ignore points farther than this (m)')
    parser.add_argument('--max-color-stddev', type=float, default=22.0,
                        help='skip pixels on strong colour edges (local 7x7 std-dev, 0-255); lower is stricter')
    parser.add_argument('--drop-uncolored', action='store_true',
                        help='write only the points that received a colour (default keeps all, uncoloured ones grey)')
    parser.add_argument('--ascii', action='store_true', help='write an ASCII PLY')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    cloud = read_cloud(args.ply)
    image = load_image(args.image)
    result = colorize_points(
        cloud.points, image, hfov_deg=args.hfov, frame=args.frame,
        max_range=args.max_range, max_color_stddev=args.max_color_stddev,
    )
    points, colors = cloud.points, result.colors
    if args.drop_uncolored:
        points, colors = points[result.confident], colors[result.confident]
    output = args.output or f'{os.path.splitext(args.ply)[0]}_colored.ply'
    write_cloud(output, Cloud(points, colors), ascii_format=args.ascii)
    s = result.stats
    print(f"{s['colored']}/{s['points']} points coloured "
          f"({s['in_view']} in view, {s['occluded']} occluded, {s['ambiguous_edges']} on colour edges).")
    print(f'Wrote {output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
