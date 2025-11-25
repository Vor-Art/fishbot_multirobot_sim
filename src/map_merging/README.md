# fishbot_map_merging

Aggregates per-robot pointcloud submaps into a voxelized global map while tracking each robot pose. The node subscribes to per-robot topics (e.g., `bot1/pose_in_world` and `bot1/pointcloud_registered`), merges them into a single voxel grid, and exposes a service to retrieve the combined map and robot poses.

## Topics

- Subscribes: `<robot>/pose_in_world` (`geometry_msgs/PoseStamped` by default, configurable via YAML).
- Subscribes: `<robot>/pointclosd_registered` (`sensor_msgs/PointCloud2`, configurable via YAML).
- Publishes (optional): `merged_map` (`sensor_msgs/PointCloud2`) if `publish_topic` is set.

## Service

- `get_merged_map` (`fishbot_map_merging/srv/GetMergedMap`): returns the merged voxel map along with the latest pose for every configured robot. Request field `clear_after_read` can be used to clear the internal voxel grid after retrieval.

## Launch

```bash
ros2 launch fishbot_map_merging map_merging.launch.py robots:=bot1,bot2
```

Parameters are loaded from `config/map_merging.yaml` by default. Adjust `robots`, `topics.pose`, `topics.pointcloud`, `voxel_size`, and `publish_topic` as needed.
