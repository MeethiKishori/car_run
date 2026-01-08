import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time


class WallFollowerAgent(Node):

    def __init__(self):
        super().__init__('wall_follower_agent')
        self.get_logger().info('Improved Wall Follower Initialized')

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/car_1/cmd_vel', 10)
        self.create_subscription(LaserScan, '/car_1/scan', self.scan_callback, 10)

        # Control parameters
        self.MAX_SPEED = 2.0
        self.MIN_SPEED = 1.0
        self.REVERSE_SPEED = 1.2

        self.TARGET_DISTANCE = 2.0
        self.BASE_GAIN = 0.7
        self.MAX_STEER = 0.5

        self.CRITICAL_SLOWDOWN = 2
        self.EXTREMELY_CLOSE = 1.5
        self.DANGER_CLOSE = 1
        self.DEADBAND = 0.05

        # State
        self.follow_left = True
        self.in_reverse = False
        self.reverse_start = None
        self.REVERSE_TIMEOUT = 1.5

        # Lidar
        self.ready = False
        self.NINETY = math.pi / 2.0

    # ------------------------------------------------------------
    # LIDAR INDEXING
    # ------------------------------------------------------------
    def configure_lidar(self, scan):
        def idx(angle):
            return int((angle - scan.angle_min) / scan.angle_increment)

        self.FRONT = idx(0.0)
        self.LEFT = idx(self.NINETY)
        self.RIGHT = idx(-self.NINETY)
        self.FRONT_L = idx(math.pi / 4)
        self.FRONT_R = idx(-math.pi / 4)

        self.ready = True
        self.get_logger().info("Lidar configured")

    def get_range(self, ranges, i):
        if i < 0 or i >= len(ranges):
            return 100.0
        window = ranges[max(0, i - 8):min(len(ranges), i + 8)]
        valid = [r for r in window if 0.05 < r < 50.0]
        return sum(valid) / len(valid) if valid else 100.0

    # ------------------------------------------------------------
    # MAIN LOOP
    # ------------------------------------------------------------
    def scan_callback(self, scan):
        if not self.ready:
            self.configure_lidar(scan)
            return

        r = scan.ranges
        front = self.get_range(r, self.FRONT)
        left = self.get_range(r, self.LEFT)
        right = self.get_range(r, self.RIGHT)
        fL = self.get_range(r, self.FRONT_L)
        fR = self.get_range(r, self.FRONT_R)

        # =========================================================
        # 1. EMERGENCY REVERSE
        # =========================================================
        danger = (
            front < self.DANGER_CLOSE or
            (left < self.DANGER_CLOSE and fR < self.DANGER_CLOSE) or
            (right < self.DANGER_CLOSE and fL < self.DANGER_CLOSE)
        )

        if danger and not self.in_reverse:
            self.in_reverse = True
            self.reverse_start = time.time()
            self.get_logger().error("EMERGENCY REVERSE")

        if self.in_reverse:
            steer = self.MAX_STEER if left > right else -self.MAX_STEER
            self.publish(-self.REVERSE_SPEED, steer)

            if time.time() - self.reverse_start > self.REVERSE_TIMEOUT and front > self.CRITICAL_SLOWDOWN:
                self.in_reverse = False
                self.follow_left = not self.follow_left
            return

        # =========================================================
        # 2. EVASION (TIGHT FRONT)
        # =========================================================
        if front < self.CRITICAL_SLOWDOWN:
            speed = self.MIN_SPEED
            steer = self.MAX_STEER if left > right else -self.MAX_STEER
            self.publish(speed, steer)
            return

        # =========================================================
        # 3. WALL FOLLOWING (P-CONTROL)
        # =========================================================
        speed = self.MAX_SPEED
        gain = self.BASE_GAIN * (2.0 if front < self.EXTREMELY_CLOSE else 1.0)

        if self.follow_left:
            error = self.TARGET_DISTANCE - left
            steer = -gain * error
        else:
            error = self.TARGET_DISTANCE - right
            steer = gain * error

        if abs(error) < self.DEADBAND:
            steer = 0.0

        steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))

        if abs(steer) > 0.2:
            speed = self.MIN_SPEED

        self.publish(speed, steer)

    # ------------------------------------------------------------
    def publish(self, speed, steer):
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steer
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = WallFollowerAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
