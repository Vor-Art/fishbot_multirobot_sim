# esdf_map

3D ESDF mapping node for multi-robot LiDAR in ROS 2. The node builds a global ESDF grid in the `map_origin` frame and exposes it via topics and a query service. It can run in two input modes:

- **Robots mode** (`input_mode:=robots`): subscribe to each robot’s registered LiDAR scan (`/botX/cloud_registered`) and use TF `map_origin -> botX/world` to transform data into the global frame.
- **Cloud mode** (`input_mode:=cloud`): subscribe to a single fused point cloud (e.g., `/global_downsampled_map`) and use TF `map_origin -> cloud_topic` to transform data into the global frame.

## ROS interfaces

- **Subscriptions**
  - `input_mode=robots`: `/botX/cloud_registered` (`sensor_msgs/PointCloud2`) per robot, configured via `bot_prefix`, `bot_ids`, and `bot_cloud_topic`. If `bot_cloud_frame` is set, the node forces that frame (usually `botX/world`); otherwise it trusts `msg.header.frame_id`.
  - `input_mode=cloud`: `/global_downsampled_map` (`sensor_msgs/PointCloud2`) or any topic set via `cloud_topic`, assumed to be in `cloud_frame` (defaults to `map_origin`).
- **TF**: `map_origin -> botX/world` (and `botX/world -> botX/lidar_link` if you later enable ray casting). Used to transform scans into `world_frame`.
- **Publications**
  - `/esdf/grid` (`sensor_msgs/PointCloud2`) – full 3D ESDF voxels in `map_origin`.
  - `/esdf/grid_roi` (`sensor_msgs/PointCloud2`) – recently updated region only.
  - `/esdf/costmap_2d` (`nav_msgs/OccupancyGrid`) – 2D ESDF slice at `costmap_layer_z`.
- **Service**
  - `/esdf/query` (`esdf_map/srv/QueryEsdf`) – returns distance and gradient at a 3D point; accepts `frame_id` for on-the-fly TF transform to `world_frame`.

## Core parameters (see `config/esdf_map.yaml`)

- **Input selection**
  - `input_mode`: `"robots"` or `"cloud"`.
  - `cloud_topic` / `cloud_frame`: fused map topic and frame for cloud mode.
  - `bot_prefix`, `bot_ids`, `bot_cloud_topic`, `bot_cloud_frame`: per-robot topic naming and optional frame override.
- **Frames / TF**
  - `world_frame`: ESDF frame (default `map_origin`).
  - `tf_timeout_sec`: TF lookup timeout.
- **Map geometry**
  - `esdf_resolution`: voxel size.
  - `map_size_x|y|z`, `map_origin_x|y|z`: ESDF extents and origin in `world_frame`.
- **Integration / ESDF**
  - `max_ray_length`, `truncation_distance`, `esdf_max_distance`, `enable_chamfer_relax`.
  - `integrate_every_cloud`: update per scan; otherwise run `esdf_update_rate_hz` timer.
- **Outputs**
  - `publish_rate_hz`: publish period for grids/costmap.
  - `publish_full_grid`, `publish_roi_grid`, `publish_costmap_2d`.
  - `costmap_layer_z`, `costmap_free_distance`, `costmap_lethal_distance`.
  - `time_log`: print rolling timing stats.

## Launch

```bash
ros2 launch esdf_map esdf_map.launch.py config_file:=/path/to/esdf_map.yaml use_sim_time:=true
```

The default `config/esdf_map.yaml` starts in fused-map (`cloud`) mode that listens to `/global_downsampled_map` from `map_fusion`. Switch to robots mode by setting `input_mode:=robots` and listing the robots in `bot_ids`.
