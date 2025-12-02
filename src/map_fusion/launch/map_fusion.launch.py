from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    root_id = int(LaunchConfiguration("root_id").perform(context))
    uav_ids_str = LaunchConfiguration("uav_ids").perform(context)
    use_sim_time_raw = LaunchConfiguration("use_sim_time").perform(context)

    use_sim_time = str(use_sim_time_raw).lower() in ("true", "1", "yes", "on")
    uav_ids = [int(x) for x in uav_ids_str.replace(" ", "").split(",") if x]

    node = Node(
        package="map_fusion",
        executable="global_map_node",
        name="global_map_node",
        output="screen",
        parameters=[{
            "root_id": root_id,
            "uav_ids": uav_ids,
            "use_sim_time": use_sim_time,
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("root_id", default_value="1", description="Root robot ID"),
        DeclareLaunchArgument("uav_ids", default_value="1,2,3,4", description="Comma-separated UAV IDs"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        OpaqueFunction(function=_launch_setup),
    ])
