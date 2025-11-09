import math

from geometry_msgs.msg import Pose


def to_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    """Create a Pose located at (x, y, z) with yaw rotation applied."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    half_yaw = yaw * 0.5
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(half_yaw)
    pose.orientation.w = math.cos(half_yaw)
    return pose


__all__ = ["to_pose"]
