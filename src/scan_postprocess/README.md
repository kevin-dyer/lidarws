# scan_postprocess

Desktop tools for scans saved by the mobile app (ASCII/binary PLY, scanner frame: x forward, y right, z down).

    colcon build --packages-select scan_postprocess && source install/setup.bash

## Colour assignment
    ros2 run scan_postprocess colorize_scan scan.ply photo.jpg -o scan_colored.ply [--hfov 65] [--yaw D --pitch D --roll D --offset X Y Z] [--frame frd|flu]

The photo is assumed to be taken at the scanner, facing +x. Only points that are in view, not occluded,
away from the photo border and not on a strong colour edge get a colour; the rest are grey
(`--drop-uncolored` removes them). `--hfov` is the field of view along the photo's *long* side (same for portrait and landscape).
A single scan cannot be calibrated reliably, so if the colours are misaligned use `--yaw/--pitch/--roll`
(degrees) and `--offset` (m) to describe the camera pose relative to the scanner.

## Registration
    ros2 run scan_postprocess register_scans a.ply b.ply [c.ply ...] -o registered.ply [--debug]

Per added cloud: FPFH+RANSAC global alignment, then multi-scale point-to-plane ICP, against everything merged
so far. Exits with code 2 and an error message if alignment is not reliable. The result opens in RViz;
`--debug` shows every stage live, one colour per scan. `--no-view` skips RViz.

## Shared scan folder (colour + register in one step)
`process_group` first estimates each photo's camera pose from photo features: SIFT matches between the
photos of overlapping scans give the relative camera geometry; together with the registered scan baseline
this pins the camera rotations and a shared focal length, and a robust reprojection check against the lidar
surface refines them. Photos that share no features with the others are fitted to the colours the calibrated
photos give the other scans. Lens distortion is not modelled (calibrate your phone lens with a checkerboard
if you need to remove it). If features cannot be matched at all it falls back to the photometric method:
`process_group` estimates each photo's camera pose (rotation, shared focal length, small offset) automatically:
it registers the scans on geometry first, then adjusts the cameras so overlapping points get consistent
brightness across the different photos. This needs at least 2 overlapping scans with photos; the search is
regularised towards the nominal pose. Use `--no-calibrate` to skip it.

    ros2 run scan_postprocess process_group ~/Downloads/scans [--debug] [-o outdir]

Finds every `.ply` under the folder (one sub-folder per scan; a photo with the same name is used if present),
colours those with a photo, then registers all of them (coloured versions preferred) into `processed/registered.ply`.
Coloured clouds are saved as `processed/<name>_colored.ply`. Scans are ordered by the timestamp in their name.

`process_group` also builds a coloured surface mesh (Poisson reconstruction, `processed/registered_mesh.ply`) and
shows it in RViz (`/registration/mesh`, "Mesh" display) next to the point cloud. Normals face the scanner that
captured each point; poorly supported or far-from-data surface is trimmed. Options: `--mesh-depth` (detail,
default 9), `--no-mesh`. Toggle the Registration/Mesh displays in RViz to compare.
