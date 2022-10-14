from tutorial_interfaces.srv import AddVec

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddVec, 'add_force_vectors', self.add_three_ints_callback)

    def add_three_ints_callback(self, request, response):
        response.sum[0] = request.a[0] + request.b[0]
        response.sum[1] = request.a[1] + request.b[1]
        response.sum[2] = request.a[2] + request.b[2]
        


        #self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()