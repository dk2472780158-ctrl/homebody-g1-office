# G1 FAST-LIO bridge

This package rotates the physically inverted G1 Mid360 point cloud and IMU by
180 degrees around X before FAST-LIO consumes them. The physical mounting TF
is kept separately, so the virtual FAST-LIO body frame remains upright.

Run the full mapping launch from the `slam` workspace:

```bash
ros2 launch g1_fastlio_bridge g1_fastlio_mapping.launch.py rviz:=true \
  map_file_path:=src/map/g1_fastlio.pcd
```

The save directory is created on demand. The map path can be relative to the
current workspace or absolute.
