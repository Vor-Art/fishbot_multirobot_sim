import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Launch Gazebo and delegate robot spawning to spawn_robots.launch.py."""
    pkg_share = get_package_share_directory('gazebo_sim')

    declare_args = [
        DeclareLaunchArgument('robot', default_value='fishbot_v2_3d'),
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('name_prefix', default_value='bot'),
        DeclareLaunchArgument('start_index', default_value='1'),
        DeclareLaunchArgument('reference_frame', default_value='world')
    ]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robots.launch.py')
        ),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'count': LaunchConfiguration('count'),
            'name_prefix': LaunchConfiguration('name_prefix'),
            'start_index': LaunchConfiguration('start_index'),
            'reference_frame': LaunchConfiguration('reference_frame')
        }.items(),
    )

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_launch)
    return ld
