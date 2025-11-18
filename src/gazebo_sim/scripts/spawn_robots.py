#!/usr/bin/env python3
"""Spawn multiple instances of a robot blueprint into Gazebo."""

from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose

from robot_utils import to_pose
from spawn_robot import GazeboSpawner, RobotBlueprint


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Spawn N instances of a gazebo_sim robot blueprint.')
    parser.add_argument('--count', type=int, default=1, help='Number of robots to spawn.')
    parser.add_argument('--robot', default='fishbot_v2_3d', help='Blueprint name (matches robots/<name> folder).')
    parser.add_argument('--name-prefix', default='bot', help='Prefix for generated entity/namespace names.')
    parser.add_argument('--start-index', type=int, default=1, help='Starting index appended to the name prefix.')
    parser.add_argument('--reference_frame', default='world', help='Reference frame to spawn.')
    parser.add_argument('--x', type=float, default=0.0, help='Base x position (meters).')
    parser.add_argument('--y', type=float, default=0.0, help='Base y position (meters).')
    parser.add_argument('--z', type=float, default=0.0, help='Base z position (meters).')
    parser.add_argument('--dx', type=float, default=0.5, help='Offset added to x for each robot.')
    parser.add_argument('--dy', type=float, default=0.0, help='Offset added to y for each robot.')
    parser.add_argument('--dz', type=float, default=0.0, help='Offset added to z for each robot.')
    parser.add_argument('--yaw', type=float, default=0.0, help='Base yaw (radians).')
    parser.add_argument('--yaw-step', type=float, default=0.0, help='Yaw offset applied per robot.')
    return parser.parse_args()


def make_pose(base_pose_args: argparse.Namespace, index: int) -> Pose:
    """Generate a Pose for the robot at a given index."""
    x = base_pose_args.x + base_pose_args.dx * index
    y = base_pose_args.y + base_pose_args.dy * index
    z = base_pose_args.z + base_pose_args.dz * index
    yaw = base_pose_args.yaw + base_pose_args.yaw_step * index
    return to_pose(x, y, z, yaw)


def main() -> int:
    args = parse_args()
    if args.count < 1:
        print('count must be >= 1', file=sys.stderr)
        return 1

    try:
        blueprint = RobotBlueprint.locate(args.robot)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1

    try:
        robot_xml = blueprint.build_description()
    except Exception as exc:  # pragma: no cover - surfaces xacro/YAML issues
        print(f'Failed to build robot description: {exc}', file=sys.stderr)
        return 1

    rclpy.init()
    node = GazeboSpawner()
    start_idx = args.start_index
    success = True

    for i in range(args.count):
        entity_name = f'{args.name_prefix}{start_idx + i}'
        pose = make_pose(args, i)
        reference_frame = args.reference_frame
        if not node.spawn(entity_name, robot_xml, pose, reference_frame):
            success = False
            break

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
