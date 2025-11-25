#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class GoalRouter(Node):
    """Routes UI goals to the currently selected robot namespace."""

    def __init__(self) -> None:
        super().__init__('goal_router')
        self.declare_parameter('active_robot', 'bot1')
        self.declare_parameter('goal_topic', 'move_base/goal')
        self.declare_parameter('ui_goal_topic', '/ui/goal_pose')

        self._active_robot = self._clean_ns(self.get_parameter('active_robot').value)
        self._goal_topic = self._clean_goal_topic(self.get_parameter('goal_topic').value)
        ui_topic = str(self.get_parameter('ui_goal_topic').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._callback_group = ReentrantCallbackGroup()
        self._goal_pub = self._create_publisher_for_robot(self._active_robot, self._goal_topic, qos)
        self._sub = self.create_subscription(
            PoseStamped,
            ui_topic,
            self._on_goal,
            qos,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._on_param_change)
        self.get_logger().info(
            f"Routing UI goals from '{ui_topic}' to '/{self._active_robot}/{self._goal_topic}'"
        )

    def _clean_ns(self, ns: str) -> str:
        return ns.lstrip('/').strip() or 'bot1'

    def _clean_goal_topic(self, name: str) -> str:
        return name.lstrip('/').strip() or 'move_base/goal'

    def _create_publisher_for_robot(self, robot: str, goal_topic: str, qos: QoSProfile):
        topic = f"/{robot}/{goal_topic}"
        return self.create_publisher(PoseStamped, topic, qos)

    def _on_goal(self, msg: PoseStamped) -> None:
        if self._goal_pub is None:
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self._goal_pub.publish(msg)

    def _on_param_change(self, params):
        new_robot: Optional[str] = None
        new_goal_topic: Optional[str] = None
        for p in params:
            if p.name == 'active_robot' and p.value is not None:
                new_robot = self._clean_ns(str(p.value))
            if p.name == 'goal_topic' and p.value is not None:
                new_goal_topic = self._clean_goal_topic(str(p.value))

        if new_robot is None and new_goal_topic is None:
            return SetParametersResult(successful=True)

        robot = new_robot or self._active_robot
        goal_topic = new_goal_topic or self._goal_topic
        if robot != self._active_robot or goal_topic != self._goal_topic:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self._goal_pub = self._create_publisher_for_robot(robot, goal_topic, qos)
            self._active_robot = robot
            self._goal_topic = goal_topic
            self.get_logger().info(f"Now routing to '/{robot}/{goal_topic}'")
        return SetParametersResult(successful=True)


def main() -> None:
    rclpy.init()
    node = GoalRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
