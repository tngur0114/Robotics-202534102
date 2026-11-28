import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class SimpleEscape(Node):
    def __init__(self):
        super().__init__('simple_escape')

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.state = "MOVE"

        self.front_threshold = 0.25
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw_tolerance = 0.05
        self.rotation_angle = math.pi / 6

        self.odom_received = False

        # 현재 로봇의 위치
        self.current_x = 0.0
        self.current_y = 0.0

        # 코너 판단에 사용하는 변수
        self.turn_count = 0
        self.corner_count = 3
        self.corner_distance = 0.30

        self.corner_x = 0.0
        self.corner_y = 0.0

        # 코너 탈출에 사용하는 변수
        self.back_x = 0.0
        self.back_y = 0.0

        self.back_distance = 0.15
        self.back_speed = 0.1
        self.rear_threshold = 0.30
        self.escape_angle = math.pi / 2

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        roll, pitch, self.current_yaw = euler_from_quaternion(quat)

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        self.odom_received = True

    def scan_callback(self, scan: LaserScan):

        if not self.odom_received:
            self.cmd_pub.publish(TwistStamped())
            return

        left_vals = [
            distance for distance in scan.ranges[:31]
            if math.isfinite(distance)
        ]
        left_avg = sum(left_vals) / len(left_vals) if left_vals else 0.0

        right_vals = [
            distance for distance in scan.ranges[-30:]
            if math.isfinite(distance)
        ]
        right_avg = sum(right_vals) / len(right_vals) if right_vals else 0.0

        front_vals = left_vals + right_vals
        front_min = min(front_vals) if front_vals else float('inf')

        # 후방 LiDAR 데이터
        rear_index = len(scan.ranges) // 2

        rear_vals = [
            distance
            for distance in scan.ranges[rear_index-30:rear_index+31]
            if math.isfinite(distance)
        ]
        rear_min = min(rear_vals) if rear_vals else 0.0

        ts = TwistStamped()

        # TURN 상태
        if self.state == "TURN":

            angle_diff = self.normalize_angle(
                self.target_yaw - self.current_yaw
            )

            if abs(angle_diff) > self.yaw_tolerance:

                ts.twist.linear.x = 0.0

                turn_speed = min(
                    self.angular_speed,
                    max(0.15, abs(angle_diff))
                )

                ts.twist.angular.z = (
                    turn_speed if angle_diff > 0
                    else -turn_speed
                )

            else:

                self.state = "MOVE"

                ts.twist.linear.x = 0.0
                ts.twist.angular.z = 0.0

        # ESCAPE 상태
        elif self.state == "ESCAPE":

            moved_distance = math.hypot(
                self.current_x - self.back_x,
                self.current_y - self.back_y
            )

            # 목표 후진 거리에 도달한 경우
            if moved_distance >= self.back_distance:

                turn_dir = 1 if left_avg > right_avg else -1

                self.target_yaw = self.normalize_angle(
                    self.current_yaw
                    + turn_dir * self.escape_angle
                )

                self.state = "TURN"
                self.turn_count = 0

                ts.twist.linear.x = 0.0
                ts.twist.angular.z = 0.0

            # 후방에 충분한 공간이 있는 경우
            elif rear_min > self.rear_threshold:

                ts.twist.linear.x = -self.back_speed
                ts.twist.angular.z = 0.0

            # 후방에 장애물이 있는 경우
            else:

                ts.twist.linear.x = 0.0
                ts.twist.angular.z = 0.0

        # MOVE 상태
        else:

            if front_min > self.front_threshold:

                ts.twist.linear.x = self.linear_speed
                ts.twist.angular.z = 0.0

            else:

                moved_distance = math.hypot(
                    self.current_x - self.corner_x,
                    self.current_y - self.corner_y
                )

                # 새로운 위치에서 장애물을 만난 경우
                if self.turn_count == 0 or moved_distance > self.corner_distance:

                    self.corner_x = self.current_x
                    self.corner_y = self.current_y
                    self.turn_count = 0

                self.turn_count += 1

                # 같은 위치에서 회피를 반복한 경우
                if self.turn_count >= self.corner_count:

                    self.state = "ESCAPE"

                    self.back_x = self.current_x
                    self.back_y = self.current_y

                    ts.twist.linear.x = 0.0
                    ts.twist.angular.z = 0.0

                # 일반적인 장애물 회피
                else:

                    self.state = "TURN"

                    turn_dir = 1 if left_avg > right_avg else -1

                    self.target_yaw = self.normalize_angle(
                        self.current_yaw
                        + turn_dir * self.rotation_angle
                    )

                    ts.twist.linear.x = 0.0
                    ts.twist.angular.z = self.angular_speed * turn_dir

        self.cmd_pub.publish(ts)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = SimpleEscape()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
