import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveRobot

class MoveRobotServer(Node):
    def __init__(self):
        super().__init__('srv_server')

        self.srv = self.create_service(MoveRobot, '/add_two_ints', self.srv_callback)
        self.get_logger().info('Srv Server Ready.')

    def srv_callback(self, request, response):
        response.sum.pose = request.a.pose + request.b.pose
        
        self.get_logger().info(f'Incoming request\nx: {request.x.pose}, y: {request.y.pose}, theta: {request.theta.pose}')
        self.get_logger().info(f'Sending back response: [{response.result_pose.pose}]')
        
        return response


def main():
    rclpy.init()
    node = MoveRobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()