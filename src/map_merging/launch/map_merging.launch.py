from __future__ import annotations

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_robot_names(raw: str) -> List[str]:
    robots = [name.strip() for name in raw.split(',') if name.strip()]
    return robots or ['bot1']


def _configure_node(context, *args, **kwargs):
    robots_value = LaunchConfiguration('robots').perform(context)
    config_file = LaunchConfiguration('config_file')
    node_namespace = LaunchConfiguration('namespace').perform(context)
    robot_names = _parse_robot_names(robots_value)

    params = [config_file, {'robots': robot_names}]
    node_kwargs = {
        'package': 'fishbot_map_merging',
        'executable': 'map_merging_node.py',
        'name': 'map_merging',
        'output': 'screen',
        'parameters': params,
    }
    if node_namespace:
        node_kwargs['namespace'] = node_namespace
    return [Node(**node_kwargs)]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots',
            default_value='bot1,bot2,bot3,bot4',
            description='Comma separated robot namespaces.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('fishbot_map_merging'),
                'config',
                'map_merging.yaml',
            ]),
            description='YAML file with default parameters.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional namespace for the map merging node.',
        ),
        OpaqueFunction(function=_configure_node),
    ])
