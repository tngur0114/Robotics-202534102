import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Processor(Node):
    def __init__(self):
        super().__init__('processor') # 'node name'
        
    
        self.subscription_ = self.create_subscription(String, '/demo_topic', self.callback, 10)
        self.subscription_
        mylocation = [0,0]

        self.publisher_ = self.create_publisher(String, '/demo_topic', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.publish_message)

    def callback(self, msg):
        if msg=='w': mylocation = mylocation + [0,1]
        if msg=='a': mylocation = mylocation + [-1,0]
        if msg=='s': mylocation = mylocation + [0,-1]
        if msg=='d': mylocation = mylocation + [1,0]

    def publish_message(self):
        self.mylocation
        msg2 = String()
        msg2.data = f"Present pose is : {self.mylocation}"
        self.publisher_.publish(msg2)
        self.get_logger().info(f'Publishing: "{msg2.data}"')

        

def main():
    rclpy.init()
    node = Processor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
