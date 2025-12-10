#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def strip_slash(frame: str) -> str:
    while frame.startswith('/'):
        frame = frame[1:]
    return frame


class GroundTruthOdom(Node):
    def __init__(self) -> None:
        super().__init__('gt_ground_truth_odom')
        robot_ids_param = self.declare_parameter('robot_ids', [1, 2]).get_parameter_value().integer_array_value
        self.robot_ids = [int(x) for x in robot_ids_param] if robot_ids_param else []
        self.robot_prefix = self.declare_parameter('robot_prefix', 'bot').get_parameter_value().string_value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.world_frame = strip_slash(self.declare_parameter('world_frame', 'world').get_parameter_value().string_value)
        self.odom_suffix = self.declare_parameter('odom_suffix', '/gt/odom').get_parameter_value().string_value
        tf_topic = self.declare_parameter('tf_topic', '/world/default/pose/info').get_parameter_value().string_value

        self.target_children = {rid: strip_slash(f'{self.robot_prefix}{rid}') for rid in self.robot_ids}
        self.pub_map = {}
        for rid in self.robot_ids:
            topic = f'/{self.robot_prefix}{rid}{self.odom_suffix}'
            self.pub_map[rid] = self.create_publisher(Odometry, topic, 10)

        if tf_topic:
            self.create_subscription(TFMessage, tf_topic, self.tf_cb, 10)
        self.get_logger().info(
            f'GT odom publisher ready for ids={self.robot_ids}, prefix={self.robot_prefix}, '
            f'world_frame={self.world_frame}'
        )

    def tf_cb(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            child = strip_slash(tf.child_frame_id.replace('::', '/'))
            parent = strip_slash(tf.header.frame_id.replace('::', '/'))
            rid = self._id_for_child(child)
            if rid is None:
                continue
            odom = Odometry()
            odom.header.stamp = tf.header.stamp
            odom.header.frame_id = self.world_frame
            odom.child_frame_id = strip_slash(f'{self.robot_prefix}{rid}/{self.base_frame}')
            odom.pose.pose.position.x = tf.transform.translation.x
            odom.pose.pose.position.y = tf.transform.translation.y
            odom.pose.pose.position.z = tf.transform.translation.z
            odom.pose.pose.orientation = tf.transform.rotation
            self.pub_map[rid].publish(odom)

    def _id_for_child(self, child: str):
        for rid, expected in self.target_children.items():
            if child == expected or child == expected + '/' + self.base_frame or child.endswith('/' + expected):
                return rid
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
