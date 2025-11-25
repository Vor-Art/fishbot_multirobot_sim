# Fishbot Foxglove Module

Foxglove-first visualization wrapper that launches `foxglove_bridge` and a
simple goal router. Set an active robot namespace once and send 2D Nav Goals
from Foxglove to that robot without spinning up per-robot RViz windows.

## Usage

```bash
source /fishbot_ws/install/setup.bash
ros2 launch fishbot_foxglove foxglove.launch.py \
  port:=8765 address:=0.0.0.0 active_robot:=bot1
```

### Launch Arguments
- `port` / `address`: WebSocket endpoint for Foxglove (defaults: 8765 / 0.0.0.0).
- `active_robot`: initial robot namespace to target (e.g., `bot1`).
- `goal_topic`: relative goal topic to publish (default: `move_base/goal`).
- `ui_goal_topic`: topic Foxglove publishes goals to (default: `/ui/goal_pose`).
- `layout`: optional path to a Foxglove layout JSON (see `share/fishbot_foxglove/layouts`).

### Goal Router

The `goal_router` node subscribes to `/ui/goal_pose` (`PoseStamped`) and
republishes to `/{active_robot}/{goal_topic}`. Update the active robot at
runtime:

```bash
ros2 param set /goal_router active_robot bot3
```

Connect Foxglove Studio to `ws://<host>:8765` and use the 2D Goal tool on the
`ui_goal_pose` topic; the router will forward it to the selected robot.

### Layouts & Robot Selector

- Layouts live in `src/foxglove_app/layouts`. A starter layout
  `multi_robot_navigation.json` includes a `VariableDropdown` to pick the active
  `robot` (default `bot0`) and uses that variable in topic names, e.g.
  `/{{robot}}/nav_path`. Load it from Foxglove Studio (File → Open Layout) or
  pass the path via launch argument: `layout:=/fishbot_ws/install/share/fishbot_foxglove/layouts/multi_robot_navigation.json`.
- The "Send Goal" publish panel in that layout targets `/ui/goal_pose`; update
  `active_robot` param on the `goal_router` node to forward goals to the chosen
  namespace.

### Compose Autolaunch (desktop or browser)

Run `docker/run_foxglove.sh` to bring up the `foxglove` compose service and
launch a client on the host automatically:

```bash
# default opens the browser at http://localhost:8765
./docker/run_foxglove.sh

# force the desktop app if installed: foxglove-studio --open ws://localhost:8765
FOXGLOVE_CLIENT=desktop ./docker/run_foxglove.sh

# skip autolaunch and only start the container
FOXGLOVE_CLIENT=none ./docker/run_foxglove.sh
```

The script reads `.env` for `FOXGLOVE_PORT`, `FOXGLOVE_ADDRESS`, and
`FOXGLOVE_CLIENT` so the same values used by `docker compose up foxglove` drive
how the host client is opened.
