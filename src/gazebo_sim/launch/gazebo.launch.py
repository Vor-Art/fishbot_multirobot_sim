import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_name = 'gazebo_sim'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_world = os.environ.get('GAZEBO_WORLD', 'fishbot.world')

    resource_paths = [
        pkg_share,
        os.path.join(pkg_share, 'worlds'),
        os.path.join(pkg_share, 'robots'),
    ]
    existing_resource = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')
    if existing_resource:
        resource_paths.append(existing_resource)
    resource_value = ':'.join(dict.fromkeys(resource_paths))

    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    world_path = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])

    start_gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', '--render-engine', 'ogre2', world_path],
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(world_arg)
    launch_description.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_value))
    launch_description.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_value))
    launch_description.add_action(start_gazebo_cmd)
    return launch_description
