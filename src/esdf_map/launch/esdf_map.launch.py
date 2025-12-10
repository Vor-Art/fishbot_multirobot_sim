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
    cloud_topic = LaunchConfiguration("cloud_topic", default="/bot1/cloud_registered")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz", default="2.0")
    costmap_resolution = LaunchConfiguration("costmap_resolution", default="0.25")
    costmap_size_m = LaunchConfiguration("costmap_size_m", default="20.0")
    default_distance = LaunchConfiguration("default_distance", default="3.0")

    esdf_node = Node(
        package="esdf_map",
        executable="esdf_map_node",
        name="esdf_map_node",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time,
                "cloud_topic": cloud_topic,
                "publish_rate_hz": publish_rate_hz,
                "costmap_resolution": costmap_resolution,
                "costmap_size_m": costmap_size_m,
                "default_distance": default_distance,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=config_file,
                              description="YAML config for esdf_map_node"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("cloud_topic", default_value="/bot1/cloud_registered",
                              description="Point cloud already in map_origin frame"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0",
                              description="Publish rate for mock outputs"),
        DeclareLaunchArgument("costmap_resolution", default_value="0.25",
                              description="Resolution of the mock occupancy grid"),
        DeclareLaunchArgument("costmap_size_m", default_value="20.0",
                              description="Side length (m) of the mock occupancy grid"),
        DeclareLaunchArgument("default_distance", default_value="3.0",
                              description="Default distance returned by the query service"),
        esdf_node,
    ])
