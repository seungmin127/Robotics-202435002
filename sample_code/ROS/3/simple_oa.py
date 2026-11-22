import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class SimpleObstacleAvoider(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoider')

        self.cmd_pub = ...
        self.scan_sub = ...

        self.front_threshold = 0.25
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def scan_callback(self, scan: LaserScan):
        left_vals = [distance for distance in scan.ranges[:31] if math.isfinite(distance)]
        left_avg = ...

        right_vals = [distance for distance in scan.ranges[-30:] if math.isfinite(distance)]
        right_avg = ...

        front_vals = left_vals + right_vals
        front_min = min(front_vals) if front_vals else float('inf')

        ts = TwistStamped()

        if ...:
            ts._ = ...
            ts._ = ...
        else:
            ts._ = ...
            ts._ = ... if left_avg > right_avg else ...

        ...  # 값이 설정된 TwistStamped 메세지를 Publish


def main():
    rclpy.init()
    node = SimpleObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
