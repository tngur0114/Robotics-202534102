import sys
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveRobot


class MoveRobotClient(Node):
    def __init__(self):
        super().__init__('move_robot_client')

        self.client = self.create_client(MoveRobot, 'move_robot')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
                    
        self.req = MoveRobot.Request()

    def send_request(self, x, y, theta, distance):
        self.req.current_pose.x = float(x)
        self.req.current_pose.y = float(y)
        self.req.current_pose.theta = float(theta)
        self.req.distance = float(distance)

        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main():
    rclpy.init()
    client = MoveRobotClient()
    response = client.send_request(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    client.get_logger().info(f'Success: {response.success}')
    client.get_logger().info(f'Result X: {response.result_pose.x:.2f}')
    client.get_logger().info(f'Result Y: {response.result_pose.y:.2f}')
    client.get_logger().info(f'Result Theta: {response.result_pose.theta:.2f}')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


