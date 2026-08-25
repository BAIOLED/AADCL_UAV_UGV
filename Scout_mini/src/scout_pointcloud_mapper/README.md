# scout_pointcloud_mapper

This node subscribes to FAST-LIO's registered world-frame scan and odometry,
filters outliers and short-lived temporal voxels, and accumulates the confirmed
static voxels. It does not publish TF and never feeds points back to FAST-LIO.

The normal one-key mapping launch starts this node and the map finalizer. Finish
the current map before stopping mapping with:

```bash
rosservice call /finish_mapping
```

This saves the filtered PCD and automatically runs `finalize_map.py`. The lower
level `/scout_pointcloud_mapper/save_map` service remains available for recovery
and diagnostics, but it does not generate the public PCD or occupancy maps.

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

The named launch above writes all results below
`~/livox_fastlio/maps/scout_map_01/`.
