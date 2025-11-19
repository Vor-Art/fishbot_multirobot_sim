import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('gazebo_sim')

    declare_args = [
        DeclareLaunchArgument('robot', default_value='fishbot_v2_3d', description='Robot blueprint folder name.'),
        DeclareLaunchArgument('count', default_value='3', description='Number of robots to spawn.'),
        DeclareLaunchArgument('name_prefix', default_value='bot', description='Prefix for spawned robot names.'),
        DeclareLaunchArgument('start_index', default_value='1', description='Starting index appended to the name prefix.'),
        DeclareLaunchArgument('reference_frame', default_value='world', description='Reference frame for spawning.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('dx', default_value='0.5'),
        DeclareLaunchArgument('dy', default_value='0.0'),
        DeclareLaunchArgument('dz', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('yaw_step', default_value='0.0'),
        DeclareLaunchArgument('spawn_delay', default_value='5.0', description='Seconds to wait before issuing spawn requests.'),
    ]

    def launch_setup(context, *args, **kwargs):
        robot = LaunchConfiguration('robot').perform(context)
        count = int(LaunchConfiguration('count').perform(context))
        name_prefix = LaunchConfiguration('name_prefix').perform(context)
        start_index = int(LaunchConfiguration('start_index').perform(context))
        reference_frame = LaunchConfiguration('reference_frame').perform(context)
        base_x = float(LaunchConfiguration('x').perform(context))
        base_y = float(LaunchConfiguration('y').perform(context))
        base_z = float(LaunchConfiguration('z').perform(context))
        dx = float(LaunchConfiguration('dx').perform(context))
        dy = float(LaunchConfiguration('dy').perform(context))
        dz = float(LaunchConfiguration('dz').perform(context))
        yaw = float(LaunchConfiguration('yaw').perform(context))
        yaw_step = float(LaunchConfiguration('yaw_step').perform(context))
        spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

        xacro_path = os.path.join(pkg_share, 'robots', robot, 'urdf', f'{robot}.xacro')
        config_path = os.path.join(pkg_share, 'robots', robot, 'config', f'{robot}.yaml')

        rsp_nodes = []
        spawn_nodes = []

        for i in range(count):
            name = f'{name_prefix}{start_index + i}'
            robot_description = Command([ 'xacro ', xacro_path,
                ' robot_namespace:=', name,
                ' config_file:=', config_path,
            ])

            rsp_nodes.append(
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    namespace=name,
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'frame_prefix': f'{name}/',
                        'robot_description': robot_description,
                    }],
                )
            )

            spawn_nodes.append(
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', name,
                        '-topic', f'/{name}/robot_description',
                        '-x', str(base_x + dx * i),
                        '-y', str(base_y + dy * i),
                        '-z', str(base_z + dz * i),
                        '-Y', str(yaw + yaw_step * i),
                        '-reference_frame', reference_frame,
                    ],
                    output='screen',
                )
            )

        if not rsp_nodes:
            return []

        spawn_after_delay = TimerAction(period=spawn_delay, actions=spawn_nodes)
        return rsp_nodes + [spawn_after_delay]

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
