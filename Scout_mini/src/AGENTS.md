# Scout Mini project instructions

## Platform

- Ubuntu 20.04, ROS Noetic, NVIDIA Jetson.
- Workspace on the robot: `~/livox_fastlio`.
- Hardware: AgileX Scout Mini, Livox Mid-360, Intel RealSense D435i.
- Build with `catkin_make -j1` unless a package has been proven safe in parallel.

## Current architecture

- `livox_ros_driver2` publishes Mid-360 LiDAR and IMU data.
- FAST-LIO supplies local odometry and registered scans. Do not modify its input
  cloud or feed a processed cloud back into it.
- `scout_pointcloud_mapper` subscribes to `/cloud_registered` and `/Odometry`,
  filters the exported mapping cloud, accumulates confirmed static voxels, and
  saves `filtered_camera_init.pcd` through
  `/scout_pointcloud_mapper/save_map`.
- FAST-LIO native PCD saving must remain disabled in both `mid360.yaml` and
  `fastlio_mapping_scout.launch`.
- `scout_map_tools/finalize_map.py` consumes the filtered PCD and generates the
  public PCD and occupancy maps.
- `fast_lio_localization` loads the public PCD and publishes `map -> odom` after
  NDT relocation.
- Navigation uses move_base, GlobalPlanner (Dijkstra), and TEB.

## TF invariants

- Every TF edge has exactly one publisher.
- `map -> odom`: NDT localization only.
- `odom -> camera_init`: Scout geometry publisher only.
- `camera_init -> body`: FAST-LIO only.
- `body -> base_link`: Scout TF manager only.
- New mapping/filter nodes must not publish TF.
- Scout wheel odometry remains a velocity source and does not publish
  `odom -> base_link` (`pub_tf=false`).

## Mapping workflow

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=scout_map_01
```

The mapper filters, accumulates, publishes, and saves automatically every 30
seconds and on normal shutdown. No save service is required. Run
`finalize_map.py` separately only when converting the finished PCD into
localization/navigation assets. Do not expect `FAST_LIO/PCD/scans.pcd`.

## Development constraints

- Preserve ROS Noetic and C++14 compatibility.
- Keep Jetson CPU, memory, and ROS bandwidth low.
- Debug point-cloud topics must be optional and disabled by default.
- Do not add nodes or topics without a concrete runtime need.
- Treat `self_filter` bounds as uncalibrated until measured on the real robot;
  it is intentionally disabled by default.
- Prefer complete-file changes and document impact before changing TF,
  extrinsics, mapping, localization, or navigation behavior.
- `WheelTech` is a separate undeveloped project and is out of scope unless the
  user explicitly requests work on it.
