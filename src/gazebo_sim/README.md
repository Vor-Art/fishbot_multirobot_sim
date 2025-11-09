# Gazebo Simulation Module

This workspace contains the `gazebo_sim` package, which keeps Gazebo worlds and robot blueprints in one place.

## Layout
- `gazebo_sim/worlds/` – ready-to-use Gazebo world files (e.g. `fishbot.world`).
- `gazebo_sim/robots/<robot_name>/` – blueprint folders; each robot has:
  - `config/*.yaml` with tunable parameters (link frames, geometry, plugin settings).
  - `urdf/*.xacro` that consumes the YAML file and builds the description used by Gazebo.
- `gazebo_sim/launch/` – launch files (e.g. `gazebo.launch.py`) that start Gazebo with the selected world.
- `gazebo_sim/scripts/` – helper utilities (currently `spawn_robot.py`).

## Parameterised robot example
`fishbot_v1` stores its main dimensions and plugin parameters in `robots/fishbot_v1/config/fishbot_v1.yaml`. The matching `robots/fishbot_v1/urdf/fishbot_v1.xacro` loads that YAML automatically, so tweaking the blueprint is as simple as editing the YAML file.

## Running Gazebo and spawning a robot
1. Build the workspace with `colcon build` and source the `install/setup.bash`.
2. Start Gazebo (defaults to `fishbot.world`, or export `GAZEBO_WORLD=<world_name>` first):
   ```bash
   ros2 launch gazebo_sim gazebo.launch.py
   ```
3. In another terminal (with the workspace sourced), spawn a blueprint instance (namespace automatically matches `--name` and `/spawn_entity` is assumed):
   ```bash
   ros2 run gazebo_sim spawn_robot.py --robot fishbot_v1 --name fishbot01 --x 0.0 --y 0.0 --z 0.0 --yaw 0.0
   ```
   Useful flags:
   - Pose options (`--x`, `--y`, `--z`, `--yaw`) to tailor the spawn request.

The helper script currently targets the default single-world Gazebo setup (service `/spawn_entity`); multi-world spawning is not yet supported.

With this setup you can add new robot blueprints by dropping another `<robot_name>` folder under `robots/`, providing its YAML config and xacro file, and then reusing the same spawn script.
