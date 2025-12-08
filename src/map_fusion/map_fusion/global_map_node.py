#!/usr/bin/env python3
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from tf2_msgs.msg import TFMessage

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


@dataclass
class BotSub:
    bot_id: int
    cloud_topic: str
    base_frame: str
    sub: any


def quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x*x + y*y + z*z + w*w
    if n < 1e-12: return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([
        [1.0 - (yy + zz),       xy - wz,       xz + wy],
        [      xy + wz, 1.0 - (xx + zz),       yz - wx],
        [      xz - wy,       yz + wx, 1.0 - (xx + yy)],
    ], dtype=float)


class MapFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('global_map_node')

        # use_sim_time may already be declared by launch parameters; ignore if so
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        self.declare_parameter('bot_prefix', 'bot')
        self.declare_parameter('bot_cloud_topic', '/cloud_registered')
        self.declare_parameter('bot_cloud_frame', 'world')
        self.declare_parameter('publish_rate_hz', 0.5)
        self.declare_parameter('combined_map_topic', '/global_downsampled_map')
        self.declare_parameter('origin_frame', 'map_origin')
        self.declare_parameter('voxel_leaf_size', 0.1)

        self.bot_prefix: str = str(self.get_parameter('bot_prefix').value)
        self.bot_cloud_topic: str = str(self.get_parameter('bot_cloud_topic').value).lstrip('/')
        self.bot_cloud_frame: str = str(self.get_parameter('bot_cloud_frame').value).lstrip('/')
        self.origin_frame_id: str = str(self.get_parameter('origin_frame').value).lstrip('/')
        self.combined_map_topic: str = str(self.get_parameter('combined_map_topic').value)
        rate_hz: float = float(self.get_parameter('publish_rate_hz').value)
        self.voxel_size: float = float(self.get_parameter('voxel_leaf_size').value)

        self._bot_re = re.compile(rf"(?:^|/){re.escape(self.bot_prefix)}(\d+)(?:/|$)")
        self._bots: Dict[int, BotSub] = {}

        qos_map = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_default = QoSProfile(depth=10)
        qos_tf = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_tf_static = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.local_maps: Dict[int, np.ndarray] = {}
        self._map_qos = qos_map

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(TFMessage, '/tf', self._on_tf_msg, qos_tf)
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_msg, qos_tf_static)

        self.global_pub = self.create_publisher(PointCloud2, self.combined_map_topic, qos_default)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._publish_global_map)

        self._pc_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.get_logger().info(
            f"global_map_node TF-fusion started. global_frame={self.origin_frame_id}, "
            f"combined_map_topic={self.combined_map_topic}, tf children=/{self.bot_prefix}<id>/{self.bot_cloud_frame}, "
            f"cloud_topic=/{self.bot_prefix}<id>/{self.bot_cloud_topic}, voxel_size={self.voxel_size}"
        )

    def _bot_id_from_frame(self, frame_id: str) -> int | None:
        f = frame_id.lstrip('/')
        m = self._bot_re.search(f)
        return int(m.group(1)) if m else None

    def _register_bot(self, bot_id: int | None) -> None:
        if bot_id is None or bot_id in self._bots:
            return

        topic = f"/{self.bot_prefix}{bot_id}/{self.bot_cloud_topic}"
        base_frame = f"{self.bot_prefix}{bot_id}/{self.bot_cloud_frame}".lstrip('/')
        sub = self.create_subscription(
            PointCloud2,
            topic,
            lambda msg, uid=bot_id: self._map_callback(uid, msg),
            self._map_qos,
        )
        self._bots[bot_id] = BotSub(bot_id=bot_id, cloud_topic=topic, base_frame=base_frame, sub=sub)
        self.get_logger().info(
            f"Discovered {self.bot_prefix}{bot_id}: subscribing to '{topic}' (frame '{base_frame}')"
        )

    def _on_tf_msg(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._register_bot(self._bot_id_from_frame(t.header.frame_id))
            self._register_bot(self._bot_id_from_frame(t.child_frame_id))

    def _map_callback(self, uid: int, msg: PointCloud2) -> None:
        pts = self._voxel_downsample(self._extract_xyz(msg))
        if pts.size == 0:
            return

        existing = self.local_maps.get(uid)
        if existing is None or existing.size == 0:
            self.local_maps[uid] = pts
        else:
            combined = np.concatenate((existing, pts), axis=0)
            self.local_maps[uid] = self._voxel_downsample(combined)

    def _extract_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Return Nx3 float32 array of xyz points; be robust to malformed input."""
        try:
            raw = np.fromiter(
                pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True),
                dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)],
            )
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"Failed to parse PointCloud2: {exc}")
            return np.empty((0, 3), dtype=np.float32)

        if raw.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        pts = np.stack((raw['x'], raw['y'], raw['z']), axis=1).astype(np.float32, copy=False)
        return pts

    def _voxel_downsample(self, pts: np.ndarray) -> np.ndarray:
        """Fast voxel filter: keep first point per voxel grid cell."""
        if pts.size == 0 or self.voxel_size <= 0.0:
            return pts

        grid = np.floor(pts / self.voxel_size).astype(np.int64)
        _, unique_idx = np.unique(grid, axis=0, return_index=True)
        return pts[unique_idx]

    def _child_frame(self, uid: int) -> str:
        bot = self._bots.get(uid)
        if bot is not None:
            return bot.base_frame
        return f"{self.bot_prefix}{uid}/{self.bot_cloud_frame}".lstrip('/')

    def _lookup_extrinsic(self, uid: int) -> Tuple[np.ndarray, np.ndarray] | None:
        if uid not in self._bots:
            return None
        child = self._child_frame(uid)
        try:
            # latest available transform
            tf = self.tf_buffer.lookup_transform(
                self.origin_frame_id,  # target (parent)
                child,                 # source (child)
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        R = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=float)
        return R, trans

    def _publish_global_map(self) -> None:
        if self.global_pub.get_subscription_count() == 0:
            return
        if not self._bots:
            return

        merged_pts: np.ndarray | None = None

        for bot in self._bots.values():
            uid = bot.bot_id
            pts = self.local_maps.get(uid)
            if pts is None or pts.size == 0:
                continue

            extr = self._lookup_extrinsic(uid)
            if extr is None:
                continue
            R, t = extr

            base_int = float(100 * (uid - 1))
            transformed = pts @ R.T + t
            transformed = transformed.astype(np.float32, copy=False)
            intensities = np.full((transformed.shape[0], 1), base_int, dtype=np.float32)
            combined = np.hstack((transformed, intensities))
            if merged_pts is None:
                merged_pts = combined
            else:
                merged_pts = np.concatenate((merged_pts, combined), axis=0)

            # Collapse accumulated data back into single ndarray to bound growth
            self.local_maps[uid] = pts

        if merged_pts is None or merged_pts.size == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.origin_frame_id
        cloud = pc2.create_cloud(header, self._pc_fields, merged_pts.tolist())
        self.global_pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
