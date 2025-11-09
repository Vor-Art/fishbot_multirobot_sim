import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'gazebo_sim'
    world_name = os.environ.get('GAZEBO_WORLD', 'fishbot.world')

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    gazebo_world_path = os.path.join(pkg_share, 'worlds', world_name)

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(start_gazebo_cmd)
    return launch_description
