import math
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveRobot


class MoveRobotServer(Node):
    def __init__(self):
        super().__init__('move_robot_server')

        self.srv = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)
        self.get_logger().info('Move Robot Service Server Started!')

    def move_robot_callback(self, request, response):
        x = request.current_pose.x
        y = request.current_pose.y
        theta = request.current_pose.theta
        distance = request.distance

        response.result_pose.x = x + distance * math.cos(theta)
        response.result_pose.y = y + distance * math.sin(theta)
        response.result_pose.theta = theta
        response.success = True

        self.get_logger().info(f'Current Pose: ({x:.2f}, {y:.2f}, {theta:.2f})')
        self.get_logger().info(f'Distance: {distance:.2f}')
        self.get_logger().info(f'Result Pose: ({response.result_pose.x:.2f}, {response.result_pose.y:.2f}, {response.result_pose.theta:.2f})')

        return response


def main():
    rclpy.init()
    node = MoveRobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

