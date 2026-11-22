import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription_ = self.create_subscription(String, '/demo_topic', self.callback, 10)
        self.subscription_

    def callback(self, msg2):
        self.get_logger().info(f'Present pose: "{msg2.data}"')

def main():
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
