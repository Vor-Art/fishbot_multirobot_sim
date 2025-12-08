import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_name = 'gazebo_sim'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_world = os.environ.get('GAZEBO_WORLD', 'fishbot.world')
    default_time_scale = os.environ.get('FISHBOT_SIM_TIME_SCALE', '1.0')

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
    time_scale_arg = DeclareLaunchArgument('sim_time_scale', default_value=default_time_scale)

    def launch_setup(context, *args, **kwargs):
        world_name = LaunchConfiguration('world').perform(context)
        sim_time_scale = float(LaunchConfiguration('sim_time_scale').perform(context))
        world_path = os.path.join(pkg_share, 'worlds', world_name)

        start_gazebo_cmd = ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', '--render-engine', 'ogre2', world_path],
            output='screen'
        )

        # Bridge simulation clock from the world topic into ROS /clock
        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            # Gazebo publishes the clock on /world/<name>/clock; remap it to /clock for ROS
            arguments=['/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            remappings=[('/world/default/clock', '/clock')],
            output='screen',
        )

        # Apply sim time scaling after Gazebo is up by calling set_physics service
        physics_req = f"real_time_factor: {sim_time_scale}"
        set_physics = TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ign', 'service',
                        '-s', '/world/default/set_physics',
                        '--reqtype', 'ignition.msgs.Physics',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '3000',
                        '--req', physics_req,
                    ],
                    output='screen',
                )
            ]
        )

        return [start_gazebo_cmd, clock_bridge, set_physics]

    launch_description = LaunchDescription()
    launch_description.add_action(world_arg)
    launch_description.add_action(time_scale_arg)
    launch_description.add_action(SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_value))
    launch_description.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_value))
    launch_description.add_action(OpaqueFunction(function=launch_setup))
    return launch_description
