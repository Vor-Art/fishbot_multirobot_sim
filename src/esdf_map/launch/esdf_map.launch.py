from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration(
        "config_file",
        default=PathJoinSubstitution([FindPackageShare("esdf_map"), "config", "esdf_map.yaml"]),
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    esdf_node = Node(
        package="esdf_map",
        executable="esdf_map_node",
        name="esdf_map_node",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file,
                              description="YAML config for esdf_map_node"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        esdf_node,
    ])
