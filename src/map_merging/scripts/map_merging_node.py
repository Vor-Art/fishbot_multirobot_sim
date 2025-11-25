#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from fishbot_map_merging.srv import GetMergedMap


@dataclass
class RobotEntry:
    pose: Optional[Pose] = None
    last_pose_time: Optional[float] = None


class RigidTransform:
    def __init__(self, rotation: Tuple[Tuple[float, float, float], ...], translation: Tuple[float, float, float]):
        self.rotation = rotation
        self.translation = translation

    @staticmethod
    def from_pose(pose: Pose) -> 'RigidTransform':
        q = pose.orientation
        rotation = _rotation_matrix(q.x, q.y, q.z, q.w)
        translation = (pose.position.x, pose.position.y, pose.position.z)
        return RigidTransform(rotation, translation)

    def transform_point(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        x, y, z = point
        r = self.rotation
        tx, ty, tz = self.translation
        rx = r[0][0] * x + r[0][1] * y + r[0][2] * z + tx
        ry = r[1][0] * x + r[1][1] * y + r[1][2] * z + ty
        rz = r[2][0] * x + r[2][1] * y + r[2][2] * z + tz
        return rx, ry, rz

    def transform_points(self, points: Iterable[Tuple[float, float, float]]) -> Iterable[Tuple[float, float, float]]:
        for point in points:
            yield self.transform_point(point)


class VoxelGrid:
    def __init__(self, voxel_size: float):
        self.voxel_size = max(voxel_size, 1e-4)
        self._cells: Dict[Tuple[int, int, int], Tuple[float, float, float, int]] = {}

    def __len__(self) -> int:
        return len(self._cells)

    def _key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        step = self.voxel_size
        return (
            int(math.floor(x / step)),
            int(math.floor(y / step)),
            int(math.floor(z / step)),
        )

    def add_points(self, points: Iterable[Tuple[float, float, float]]) -> int:
        added = 0
        for x, y, z in points:
            key = self._key(x, y, z)
            sx, sy, sz, count = self._cells.get(key, (0.0, 0.0, 0.0, 0))
            self._cells[key] = (sx + x, sy + y, sz + z, count + 1)
            added += 1
        return added

    def clear(self) -> None:
        self._cells.clear()

    def to_pointcloud(self, frame_id: str, stamp) -> PointCloud2:
        header = Header()
        header.frame_id = frame_id
        header.stamp = stamp
        points = []
        for sx, sy, sz, count in self._cells.values():
            inv = 1.0 / float(max(count, 1))
            points.append((sx * inv, sy * inv, sz * inv))
        return point_cloud2.create_cloud_xyz32(header, points)


def _rotation_matrix(x: float, y: float, z: float, w: float) -> Tuple[Tuple[float, float, float], ...]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


class MapMergingNode(Node):
    def __init__(self) -> None:
        super().__init__('map_merging')
        self.declare_parameter('robots', ['bot1'])
        self.declare_parameter('topics.pose', 'pose_in_world')
        self.declare_parameter('topics.pointcloud', 'pointclosd_registered')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('publish_topic', 'merged_map')
        self.declare_parameter('publish_period', 1.0)

        robots_param = self.get_parameter('robots').value
        self.robot_names = self._parse_robot_list(robots_param)
        self.pose_topic = str(self.get_parameter('topics.pose').value or 'pose_in_world')
        self.cloud_topic = str(self.get_parameter('topics.pointcloud').value or 'pointclosd_registered')
        self.world_frame = str(self.get_parameter('world_frame').value or 'world')
        self.publish_topic = str(self.get_parameter('publish_topic').value or '')
        self.publish_period = float(self.get_parameter('publish_period').value or 0.0)

        voxel_size = float(self.get_parameter('voxel_size').value)
        self.voxel_grid = VoxelGrid(voxel_size)
        self.robot_states: Dict[str, RobotEntry] = {name: RobotEntry() for name in self.robot_names}
        self._last_warn: Dict[str, float] = {}

        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        for name in self.robot_names:
            pose_topic = f"{name}/{self.pose_topic}"
            cloud_topic = f"{name}/{self.cloud_topic}"
            self.create_subscription(PoseStamped, pose_topic, lambda msg, robot=name: self._on_pose(robot, msg), pose_qos)
            self.create_subscription(PointCloud2, cloud_topic, lambda msg, robot=name: self._on_pointcloud(robot, msg), cloud_qos)
            self.get_logger().info(f"Listening for {name}: pose '{pose_topic}', pointcloud '{cloud_topic}'")

        self.create_service(GetMergedMap, 'get_merged_map', self._on_get_merged_map)

        self.map_pub = None
        self.publish_timer = None
        if self.publish_topic:
            self.map_pub = self.create_publisher(PointCloud2, self.publish_topic, 10)
            if self.publish_period > 0.0:
                self.publish_timer = self.create_timer(self.publish_period, self._publish_periodic_map)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _parse_robot_list(value: Sequence[str] | str) -> List[str]:
        if isinstance(value, str):
            items = [item.strip() for item in value.split(',') if item.strip()]
            return items or ['bot1']
        if isinstance(value, Sequence):
            return [str(item) for item in value if str(item)]
        return ['bot1']

    def _throttled_warn(self, key: str, message: str, interval: float = 5.0) -> None:
        now = time.monotonic()
        last = self._last_warn.get(key, 0.0)
        if now - last >= interval:
            self._last_warn[key] = now
            self.get_logger().warn(message)

    def _on_pose(self, robot: str, msg: PoseStamped) -> None:
        entry = self.robot_states.setdefault(robot, RobotEntry())
        entry.pose = msg.pose
        entry.last_pose_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_pointcloud(self, robot: str, msg: PointCloud2) -> None:
        entry = self.robot_states.get(robot)
        pose = entry.pose if entry else None
        if pose is None:
            self._throttled_warn(robot, f"No pose for {robot}; skipping incoming pointcloud.")
            return

        transform = RigidTransform.from_pose(pose)
        points_iter = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        added = self.voxel_grid.add_points(transform.transform_points(points_iter))
        if added > 0:
            self.get_logger().debug(f"Integrated {added} points from {robot} into voxel grid.")

    def _publish_periodic_map(self) -> None:
        if not self.map_pub:
            return
        cloud = self._build_pointcloud()
        self.map_pub.publish(cloud)

    def _build_pointcloud(self) -> PointCloud2:
        stamp = self.get_clock().now().to_msg()
        return self.voxel_grid.to_pointcloud(self.world_frame, stamp)

    def _on_get_merged_map(self, request: GetMergedMap.Request, response: GetMergedMap.Response) -> GetMergedMap.Response:
        cloud = self._build_pointcloud()
        response.robot_names = list(self.robot_states.keys())
        response.poses = [entry.pose or Pose() for entry in self.robot_states.values()]
        response.merged_map = cloud

        has_map = len(self.voxel_grid) > 0
        has_pose = any(entry.pose is not None for entry in self.robot_states.values())
        response.success = has_map or has_pose
        response.message = (
            f"Merged {len(self.voxel_grid)} voxels from {len(self.robot_states)} robots."
            if response.success else "No robot data received yet."
        )

        if request.clear_after_read:
            self.voxel_grid.clear()
        return response


def main() -> None:
    rclpy.init()
    node = MapMergingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
