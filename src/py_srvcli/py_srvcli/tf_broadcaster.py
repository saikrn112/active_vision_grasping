import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose

class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1, self.handle_turtle_pose)

    def handle_turtle_pose(self):

        position_x = 0.0
        position_y = 0.0
        position_z = 20.0

        quat_x = 0.0
        quat_y = 0.0
        quat_z = 0.0
        quat_w = 1.0

        t = TransformStamped()    # container for message

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = "world"
        t.transform.translation.x = position_x
        t.transform.translation.y = position_y
        t.transform.translation.z = position_z
        t.transform.rotation.x = quat_x
        t.transform.rotation.y = quat_y
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w

        self.tf_broadcaster.sendTransform(t)
        return True



def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
