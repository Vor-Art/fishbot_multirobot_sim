# ROS 2 ESDF node (C++ scaffold)

This package now ships a lightweight C++ node that mimics the ESDF processing pipeline so you can wire up the interfaces before the real mapping code lands.

## Interfaces
- `/esdf/grid` (`sensor_msgs/msg/PointCloud2`): republishes the latest input cloud in `map_origin`
  as a stand-in for a true ESDF point cloud.
- `/esdf/costmap_2d` (`nav_msgs/msg/OccupancyGrid`): procedural mock costmap with a simple ripple
  pattern; replace with a real 2D projection later.
- `/esdf/query` (`esdf_map/srv/QueryEsdf`): responds with a mocked distance/gradient pair so the
  planner API can be integrated early.

Incoming data is consumed from `cloud_topic` (defaults to `/bot1/cloud_registered`) using
`SensorDataQoS`.

## Parameters
- `cloud_topic` (string): source cloud topic, already in the **map** frame.
- `publish_rate_hz` (double): how often to publish the mock outputs.
- `costmap_resolution` (double): resolution for the generated occupancy grid.
- `costmap_size_m` (double): side length of the square mock costmap.
- `default_distance` (double): baseline distance returned by the query service when no ESDF exists.

## Config file
Default parameters live in `esdf_map/config/esdf_map.yaml`. The launch file loads it automatically
and still lets you override individual parameters via launch arguments, e.g.:
```bash
ros2 launch esdf_map esdf_map.launch.py costmap_resolution:=0.2
```

## Build and run
```bash
colcon build --packages-select esdf_map
source install/setup.bash
ros2 launch esdf_map esdf_map.launch.py
```

Everything is currently mocked; swap the internals of `EsdfMapNode` with real ESDF plumbing without changing the public interfaces.
