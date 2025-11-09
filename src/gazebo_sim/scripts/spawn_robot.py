#!/usr/bin/env python3
"""Spawn a robot blueprint into a running (single-world) Gazebo instance."""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import xacro
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node

from robot_utils import to_pose

@dataclass(frozen=True)
class RobotBlueprint:
    name: str
    xacro_path: Path
    config_path: Path

    @classmethod
    def locate(cls, robot: str) -> 'RobotBlueprint':
        pkg_share = Path(get_package_share_directory('gazebo_sim'))
        robot_dir = pkg_share / 'robots' / robot
        xacro_path = robot_dir / 'urdf' / f'{robot}.xacro'
        config_path = robot_dir / 'config' / f'{robot}.yaml'

        if not xacro_path.exists():
            raise FileNotFoundError(f'Could not find xacro file at {xacro_path}')
        if not config_path.exists():
            raise FileNotFoundError(f'Could not find config file at {config_path}')
        return cls(robot, xacro_path, config_path)

    def build_description(self) -> str:
        doc = xacro.process_file(str(self.xacro_path), mappings={'config_file': str(self.config_path)})
        return doc.toprettyxml(indent='  ')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Spawn a gazebo_sim robot blueprint into Gazebo.')
    parser.add_argument('--robot', required=True, help='Blueprint name (matches robots/<name> folder).')
    parser.add_argument('--name', required=True, help='Entity/namespace name for Gazebo and ROS topics.')
    parser.add_argument('--reference_frame', default='world', help='Reference frame to spawn.')
    parser.add_argument('--x', type=float, default=0.0, help='Initial x position (meters).')
    parser.add_argument('--y', type=float, default=0.0, help='Initial y position (meters).')
    parser.add_argument('--z', type=float, default=0.0, help='Initial z position (meters).')
    parser.add_argument('--yaw', type=float, default=0.0, help='Initial yaw (radians).')
    return parser.parse_args()


class GazeboSpawner(Node):
    """Lightweight helper that calls /spawn_entity once."""

    def __init__(self) -> None:
        super().__init__('gazebo_robot_spawner')
        self._client = self.create_client(SpawnEntity, '/spawn_entity')

    def spawn(self, name: str, xml: str, pose: Pose, reference_frame: str) -> bool:
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/spawn_entity service not available.')
                return False

        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.robot_namespace = name
        request.reference_frame = reference_frame
        request.initial_pose = pose

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned '{name}' via /spawn_entity.")
            return True

        self.get_logger().error(f'Failed to spawn: {future.exception()}')
        return False


def main() -> int:
    args = parse_args()

    blueprint = RobotBlueprint.locate(args.robot)
    robot_xml = blueprint.build_description()

    rclpy.init()
    node = GazeboSpawner()
    pose = to_pose(args.x, args.y, args.z, args.yaw)
    success = node.spawn(args.name, robot_xml, pose)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
