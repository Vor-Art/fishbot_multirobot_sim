import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Start Gazebo and spawn three robots."""
    pkg_share = get_package_share_directory('gazebo_sim')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_robots = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_sim', 'spawn_robots.py', '--count', '3'],
        output='screen',
    )

    # Give Gazebo a moment to start before spawning robots.
    spawn_after_gazebo = TimerAction(period=5.0, actions=[spawn_robots])

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_after_gazebo)
    return ld
