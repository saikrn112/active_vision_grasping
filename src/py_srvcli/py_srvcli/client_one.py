#from ssl import _SrvnmeCbType
import sys

from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped
import tf2_ros

import rclpy
from rclpy.node import Node
import numpy as np


class SetEntityClientAsync(Node):

    def __init__(self):
        super().__init__('set_entity_client_async')

        self.cli = self.create_client(SetEntityState, '/demo/set_entity_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = SetEntityState.Request()

        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(1, self.publish_transform)

    def publish_transform(self):
        position_x = 1.0
        position_y = 0.0
        position_z = 1.0

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

    def send_request(self):
        position_x = 1.0
        position_y = 0.0
        position_z = 1.0

        quat_x = 0.0
        quat_y = 0.0
        quat_z = 0.0
        quat_w = 1.0


        self.req.state.name = "camera"
        self.req.state.pose.position.x = position_x
        self.req.state.pose.position.y = position_y
        self.req.state.pose.position.z = position_z

        self.req.state.pose.orientation.x = quat_x
        self.req.state.pose.orientation.y = quat_y
        self.req.state.pose.orientation.z = quat_z
        self.req.state.pose.orientation.w = quat_w

        self.future = self.cli.call_async(self.req)
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

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main():
    rclpy.init()
    set_entity_client = SetEntityClientAsync()
    response = set_entity_client.send_request()
    print(response)

    try:
        rclpy.spin(set_entity_client)
    except KeyboardInterrupt:
        pass


    set_entity_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
