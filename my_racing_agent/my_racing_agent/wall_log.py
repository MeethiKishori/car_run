import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import csv
import os


class PIDWallFollower(Node):
    """
    Clean PID Wall Follower Controller with CSV logging
    - PID controller with buffered CSV telemetry
    """
    def __init__(self):
        super().__init__('pid_wall_follower')
        self.get_logger().info('PID Wall Follower Initialized')

        # Publishers and Subscribers
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  #original
        self.scan_subscriber = self.create_subscription( LaserScan, '/scan', self.scan_callback, 10) #original
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) #original  

        #self.drive_publisher = self.create_publisher(Twist, '/car_1/cmd_vel', 10)
        #self.scan_subscriber = self.create_subscription(LaserScan, '/car_1/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Controller selection
        #self.controller_mode = 'P'  # Active P
        #self.controller_mode = 'PI' # Active PI
        self.controller_mode = 'PID' # Active PID

        # PID Gains
        self.Kp = 0.8
        self.Ki = 0.1
        self.Kd = 0.5

        # Control Parameters
        self.TARGET_DISTANCE = 1.0
        self.MAX_SPEED = 2.0
        self.MIN_SPEED = 0.5
        self.MAX_ANGULAR = 0.6

        # Collision thresholds
        self.FRONT_DANGER = 0.6
        self.SIDE_DANGER = 0.4

        # Wall following
        self.follow_left = True

        # PID State
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.integral_max = 1.0
        self.integral_min = -1.0

        # Lidar indices
        self.FRONT_INDEX = None
        self.LEFT_INDEX = None
        self.RIGHT_INDEX = None
        self.is_config_loaded = False

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ============================================================
        # CSV LOGGING SETUP
        # ============================================================
        self.csv_file_path = os.path.join(os.getcwd(), 'src', 'my_racing_agent', 'my_racing_agent', 'pid.csv') #choose p.csv or pid.csv
        self.csv_headers = [
            'timestamp', 'x', 'y', 'theta', 'linear_speed', 'angular_speed',
            'angular_cmd', 'speed_cmd', 'cte', 'safety_flag'
        ]
        self.csv_buffer = []
        self.buffer_size = 50  # Flush every 50 samples

        #if not os.path.isfile(self.csv_file_path): --- IGNORing will replace whole file in every run---
        with open(self.csv_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.csv_headers)

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def calculate_lidar_indices(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)

        def angle_to_index(angle):
            idx = int((angle - angle_min) / angle_increment)
            return max(0, min(num_readings - 1, idx))

        self.FRONT_INDEX = angle_to_index(0.0)
        self.LEFT_INDEX = angle_to_index(math.pi / 2.0)
        self.RIGHT_INDEX = angle_to_index(-math.pi / 2.0)

        self.get_logger().info(
            f'Lidar configured: Front={self.FRONT_INDEX}, Left={self.LEFT_INDEX}, Right={self.RIGHT_INDEX}'
        )
        self.is_config_loaded = True

    def get_range(self, ranges, index, window=10):
        if index is None:
            return 100.0
        num_readings = len(ranges)
        start = max(0, index - window)
        end = min(num_readings, index + window + 1)
        segment = ranges[start:end]
        valid = [r for r in segment if 0.1 < r < 30.0 and not math.isinf(r)]
        return sum(valid) / len(valid) if valid else 100.0

    def compute_control(self, error, dt):
        if self.controller_mode == 'P':
            control = self.Kp * error
            self.get_logger().info(
                f'[P] Error={error:.3f}, Control={control:.3f}'
            )
        elif self.controller_mode == 'PI':
            self.integral += error * dt
            self.integral = max(self.integral_min, min(self.integral_max, self.integral))
            control = self.Kp * error + self.Ki * self.integral
            self.get_logger().info(
                f'[PI] Error={error:.3f}, Integral={self.integral:.3f}, Control={control:.3f}'
            )
        elif self.controller_mode == 'PID':
            self.integral += error * dt
            self.integral = max(self.integral_min, min(self.integral_max, self.integral))
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.previous_error = error
            self.get_logger().info(
                f'[PID] Error={error:.3f}, Integral={self.integral:.3f}, '
                f'Derivative={derivative:.3f}, Control={control:.3f}'
            )
        else:
            control = 0.0

        return max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, control))

    def log_to_buffer(self, angular_cmd, speed_cmd, cte, safety_flag):
        """Append data to buffer and flush if full"""
        timestamp = time.time()
        row = [
            timestamp, self.x, self.y, self.theta,
            speed_cmd, angular_cmd, angular_cmd, speed_cmd,
            cte, safety_flag
        ]
        self.csv_buffer.append(row)

        if len(self.csv_buffer) >= self.buffer_size:
            self.flush_csv_buffer()

    def flush_csv_buffer(self):
        if self.csv_buffer:
            with open(self.csv_file_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.csv_buffer)
            self.csv_buffer = []

    def scan_callback(self, msg: LaserScan):
        if not self.is_config_loaded:
            self.calculate_lidar_indices(msg)
            return

        ranges = msg.ranges
        front = self.get_range(ranges, self.FRONT_INDEX)
        left = self.get_range(ranges, self.LEFT_INDEX)
        right = self.get_range(ranges, self.RIGHT_INDEX)

        cmd = Twist()
        safety_flag = 0  # 0 = normal, 1 = front danger, 2 = side danger

        # Front collision
        if front < self.FRONT_DANGER:
            cmd.linear.x = -self.MIN_SPEED
            cmd.angular.z = self.MAX_ANGULAR if left > right else -self.MAX_ANGULAR
            self.integral = 0.0
            safety_flag = 1
            self.drive_publisher.publish(cmd)
            self.log_to_buffer(cmd.angular.z, cmd.linear.x, 0.0, safety_flag)
            return

        # Side collision
        if (self.follow_left and left < self.SIDE_DANGER) or \
           (not self.follow_left and right < self.SIDE_DANGER):
            cmd.linear.x = self.MIN_SPEED
            cmd.angular.z = -self.MAX_ANGULAR if self.follow_left else self.MAX_ANGULAR
            self.integral = 0.0
            safety_flag = 2
            self.drive_publisher.publish(cmd)
            self.log_to_buffer(cmd.angular.z, cmd.linear.x, 0.0, safety_flag)
            return

        # Compute CTE
        if self.follow_left:
            cte = self.TARGET_DISTANCE - left
        else:
            cte = right - self.TARGET_DISTANCE

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        angular_velocity = self.compute_control(cte, dt)

        # Speed adjustment
        turn_ratio = abs(angular_velocity) / self.MAX_ANGULAR
        if turn_ratio > 0.5:
            speed = self.MIN_SPEED
        elif turn_ratio > 0.3:
            speed = (self.MAX_SPEED + self.MIN_SPEED) / 2.0
        else:
            speed = self.MAX_SPEED
        if front < 2.0:
            speed = min(speed, self.MIN_SPEED)

        cmd.linear.x = speed
        cmd.angular.z = -angular_velocity if self.follow_left else angular_velocity
        self.drive_publisher.publish(cmd)

        # Log telemetry
        self.log_to_buffer(cmd.angular.z, cmd.linear.x, cte, safety_flag)

    def destroy_node(self):
        """Flush remaining CSV buffer on shutdown"""
        self.flush_csv_buffer()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDWallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop = Twist()
    node.drive_publisher.publish(stop)
    node.destroy_node()
    rclpy.shutdown()
