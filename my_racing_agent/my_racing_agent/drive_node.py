import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class WallFollowerAgent(Node):

    def __init__(self):
        super().__init__('wall_follower_agent')

        self.get_logger().info("Simple Wall Follower Initialized")

        #self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #for rviz
        self.cmd_pub = self.create_publisher(Twist, "/car_1/cmd_vel", 10) #for gazebo

        #self.create_subscription(LaserScan, "/scan", self.scan_callback, 10) #for rviz
        self.create_subscription(LaserScan, "/car_1/scan", self.scan_callback, 10) #for gazebo
        #self.create_subscription(Odometry, "/odom", self.odom_callback, 10) #for rviz
        self.create_subscription(Odometry, "/car_1/ground_truth", self.odom_callback, 10) #for gazebo

        # Speed
        self.MAX_SPEED = 2.5
        self.CRUISE_SPEED = 1.0
        self.MIN_SPEED = 0.4

        # Distances
        self.TARGET_DISTANCE = 1.5
        self.FRONT_DANGER = 1.2
        self.SIDE_TOO_CLOSE = 0.5

        # PID
        self.P_GAIN = 1.5
        self.D_GAIN = 0.25
        self.MAX_STEER = 1.0
        self.DEADBAND = 0.04

        self.prev_error = 0.0

        self.follow_left = True

        self.ready = False
        self.NINETY = math.pi / 2

    def configure_lidar(self, scan):

        def idx(angle):
            i = int((angle - scan.angle_min) / scan.angle_increment)
            return max(0, min(len(scan.ranges) - 1, i))

        self.FRONT = idx(0.0)
        self.LEFT = idx(self.NINETY)
        self.RIGHT = idx(-self.NINETY)

        self.ready = True

        self.get_logger().info("Lidar configured")

    def get_range(self, ranges, i, window=6):

        if i < 0 or i >= len(ranges):
            return 100.0

        seg = ranges[max(0, i - window): min(len(ranges), i + window)]

        valid = [r for r in seg if 0.05 < r < 50 and not math.isinf(r)]

        if not valid:
            return 100.0

        return sum(valid) / len(valid)

    def odom_callback(self, msg):
        pass

    def scan_callback(self, scan):

        if not self.ready:
            self.configure_lidar(scan)
            return

        r = scan.ranges

        front = self.get_range(r, self.FRONT)
        left = self.get_range(r, self.LEFT)
        right = self.get_range(r, self.RIGHT)

        # ------------------------------------------------
        # FRONT OBSTACLE AVOIDANCE
        # ------------------------------------------------

        if front < self.FRONT_DANGER:

            self.get_logger().warn(f"Front obstacle {front:.2f}m")

            if right > left:
                steer = -self.MAX_STEER
            else:
                steer = self.MAX_STEER

            self.publish(self.MIN_SPEED, steer)
            return

        # ------------------------------------------------
        # WALL FOLLOW PID
        # ------------------------------------------------

        if self.follow_left:
            error = self.TARGET_DISTANCE - left
        else:
            error = self.TARGET_DISTANCE - right

        p = self.P_GAIN * error
        d = self.D_GAIN * (error - self.prev_error)

        self.prev_error = error

        steer = p + d

        if abs(error) < self.DEADBAND:
            steer = 0.0

        steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))

        speed = self.CRUISE_SPEED

        if abs(steer) > 0.4:
            speed = self.MIN_SPEED

        # ------------------------------------------------
        # SIDE SAFETY
        # ------------------------------------------------

        if self.follow_left and left < self.SIDE_TOO_CLOSE:
            steer = -self.MAX_STEER

        if not self.follow_left and right < self.SIDE_TOO_CLOSE:
            steer = self.MAX_STEER

        self.publish(speed, steer)

    def publish(self, speed, steer):

        cmd = Twist()

        cmd.linear.x = float(speed)
        cmd.angular.z = float(steer)

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


if __name__ == "__main__":
    main()