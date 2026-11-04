"""mesh_scan: create a surface mesh from a point cloud PLY using Poisson reconstruction."""
from __future__ import annotations

import argparse
import os
import sys

from .meshing import build_mesh, write_mesh
from .ply_io import read_cloud


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='mesh_scan',
        description='Create a surface mesh from a point cloud using Poisson surface reconstruction.',
    )
    parser.add_argument('ply', help='input point cloud (.ply)')
    parser.add_argument('-o', '--output', help='output mesh (.ply or .obj) (default: <input>_mesh.ply)')
    parser.add_argument('--voxel', type=float, default=0.03,
                        help='voxel size for normal estimation in meters (default 0.03)')
    parser.add_argument('--depth', type=int, default=9,
                        help='octree depth for reconstruction: higher=more detail but slower, 8-10 typical (default 9)')
    parser.add_argument('--trim-density', type=float, default=0.08,
                        help='remove triangles with density < quantile (0-1); higher=more aggressive (default 0.08)')
    parser.add_argument('--max-gap-voxels', type=float, default=4.0,
                        help='remove vertices >this many voxels from any point cloud (default 4.0)')
    parser.add_argument('--ascii', action='store_true', help='write ASCII mesh')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    # Load point cloud
    print(f'Loading {args.ply}...')
    cloud = read_cloud(args.ply)
    print(f'Loaded {len(cloud.points)} points')

    # Build mesh
    print(f'Building mesh with depth={args.depth}, voxel={args.voxel}m, trim_density={args.trim_density}...')
    mesh = build_mesh(
        cloud,
        voxel=args.voxel,
        depth=args.depth,
        trim_density=args.trim_density,
        max_gap_voxels=args.max_gap_voxels,
    )
    print(f'Mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles')

    # Write mesh
    output = args.output or f'{os.path.splitext(args.ply)[0]}_mesh.ply'
    print(f'Writing {output}...')
    write_mesh(output, mesh)
    print(f'Wrote {output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
