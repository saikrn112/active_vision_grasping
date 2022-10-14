#from ssl import _SrvnmeCbType
import sys

from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped
import tf2_ros

import rclpy
from rclpy.node import Node
import numpy as np


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetEntityState, '/demo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetEntityState.Request()
        self.br = tf2_ros.TransformBroadcaster(self)

    def send_request(self, a, b):
        self.req.state.name = "camera"
        position_x = 0.0
        position_y = 0.0
        position_z = 20.0

        quat_x = 0.0
        quat_y = 0.0
        quat_z = 0.0
        quat_w = 0.0

        self.req.state.pose.position.x = position_x
        self.req.state.pose.position.y = position_y
        self.req.state.pose.position.z = position_z

        self.req.state.pose.orientation.x = quat_x
        self.req.state.pose.orientation.y = quat_y
        self.req.state.pose.orientation.z = quat_z
        self.req.state.pose.orientation.w = quat_w

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

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

        self.br.sendTransform(t)
        return self.future.result()


def main():
    rclpy.init()
    a = [5,1,0]
    b = [3,4,6]
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(a,b)
    print(response)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
