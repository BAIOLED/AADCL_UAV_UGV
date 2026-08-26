# scout_pointcloud_mapper

This node subscribes to FAST-LIO's registered world-frame scan and odometry,
filters outliers and short-lived temporal voxels, and accumulates the confirmed
static voxels. It does not publish TF and never feeds points back to FAST-LIO.

The normal mapping launch starts this node automatically. It saves the filtered
PCD every 30 seconds and once more during a normal shutdown, so no save service
is needed in the normal workflow. `/scout_pointcloud_mapper/save_map` remains
available for diagnostics only.

Reset all candidate and confirmed voxels:

```bash
rosservice call /scout_pointcloud_mapper/reset_map
```

Start a named mapping session with:

```bash
roslaunch scout_system_bringup scout_mapping.launch map_name:=scout_map_01
```

The default mapping launch writes to:

```text
~/livox_fastlio/maps/current_mapping/filtered_camera_init.pcd
```

The named launch writes `filtered_camera_init.pcd` below
`~/livox_fastlio/maps/scout_map_01/`. Run `finalize_map.py` once when converting
that PCD into localization and navigation assets.
