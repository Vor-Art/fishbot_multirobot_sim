# esdf_map

3D ESDF mapping node for multi-robot LiDAR in ROS 2.
The node fuses point clouds from multiple `botX` robots into a single global ESDF grid in the `map_origin` frame and exposes it via topics and a query service.
Internally, the core follows a TSDF+ESDF pipeline similar to Voxblox / ROG-Map / Fast-Planner SDFMap, but wrapped in a ROS-agnostic `EsdfMapCore` class.

## ROS interfaces

### Subscriptions

- `/botX/cloud_registered` (`sensor_msgs/PointCloud2`)

  One topic per robot, constructed from:
  - `bot_prefix` (e.g. `"bot"`)
  - `bot_ids` (e.g. `[1, 2, 3]`)
  - `bot_cloud_topic` (e.g. `"cloud_registered"`)

### TF

Required transforms:

- `map_origin -> botX/world`
- `botX/world -> botX/lidar_link`

The node uses these to express every scan and sensor pose in `map_origin`.

### Publications

- `/esdf/grid` - 3D ESDF voxel grid in `map_origin` (distance field around obstacles).
- `/esdf/costmap_2d` (`nav_msgs/OccupancyGrid`) - 2D ESDF slice at height `costmap_layer_z` for RViz and 2D planners.

### Services

- `/esdf/query` - Query distance and gradient at a 3D point in `map_origin` (or a given frame).

## Parameters

All parameters are in `config/esdf_map.yaml`.

- **Robot IO**
  - `bot_prefix` – prefix before robot id (e.g. `"bot"`).
  - `bot_ids` – list of robot numeric ids.
  - `bot_cloud_topic` – relative topic name for registered cloud.
  - `bot_sensor_frame` – lidar frame under each robot.
  - `world_frame` – global map frame (`"map_origin"`).

- **Map geometry**
  - `esdf_resolution` – voxel size [m].
  - `map_size_x`, `map_size_y`, `map_size_z` – ESDF extent [m].
  - `map_origin_x/y/z` – ESDF origin in `world_frame`.

- **Integration & ESDF**
  - `max_ray_length` – max ray length from sensor [m].
  - `truncation_distance` – TSDF truncation distance [m].
  - `esdf_max_distance` – clamp ESDF values [m].
  - `integrate_every_cloud` – update ESDF on every scan or by timer.
  - `esdf_update_rate_hz` – ESDF update rate if not per scan.

- **Output**
  - `publish_rate_hz` – grid / costmap publish rate [Hz].
  - `publish_full_grid` – enable `/esdf/grid`.
  - `publish_costmap_2d` – enable 2D slice.
  - `costmap_layer_z` – slice height [m].
  - `costmap_free_distance` / `costmap_lethal_distance` – ESDF thresholds for 2D costmap.

## Usage

Build the package, then:

```bash
ros2 launch esdf_map esdf_map.launch.py
