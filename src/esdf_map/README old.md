# Ros2 ESDF3d node

This module should incrimently build the global ESDF for the entire environment. ESDF map is constantly updated as the robot moves, adding new information. 

## The ESDF3d mapping node operates by subscribing to point cloud data:

- `/botX/cloud_registered` - The current PointCloud2 scan form the **botX** that already transformated in the `map_origin` frame. The node then processes these points and transforms them into the ESDF.

- Also it should get the transformation from tf2, where glogal frame is `map_origin` and robot's lidar frame is `botX/lidar_link`.

## This node should publishe the following:

- `/esdf/grid`: This is the core output – the 3D ESDF map itself. It's published in either the `map_origin` frame. This is the data that the planner uses to understand the environment. Think of it as the robot's "mental model" of the surroundings.

- `/esdf/query`: This provides a service that allows other nodes (specifically the planner) to query the ESDF for distance and gradient information. The planner can ask questions like "What's the distance to the nearest obstacle at this point?" or "Which direction should I move to avoid something?"

- `/esdf/costmap_2d`: This is a 2D projection of the ESDF, often used for visualization or for legacy planning approaches. It simplifies the 3D map into a 2D representation. This might be helpful for certain planning algorithms or for visualizing the robot's understanding of its surroundings.
