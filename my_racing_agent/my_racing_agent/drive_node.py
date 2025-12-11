import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math


class WallFollowerAgent(Node):
    """
    Stable Wall Follower Agent with dynamic 90-degree indices, using P-control, 
    and enhanced safety logic that doubles P-control gain when approaching a wall 
    head-on (Front < EXTREMELY_CLOSE).
    """
    def __init__(self):
        super().__init__('wall_follower_agent')
        self.get_logger().info('Wall Follower Agent Node Initialized (Dynamic P-Gain Added).')

        # --- Publishers and Subscribers ---
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- Controller Parameters ---
        self.MAX_SPEED = 1             # Max cruising speed
        self.MIN_SPEED = 0.2           # Slow cornering/safety speed
        self.TARGET_DISTANCE = 1.0     # Fixed Target Distance (1 meter)
        self.STEERING_GAIN = 0.7       # Proportional Gain (Base Gain)
        self.MAX_STEER = 0.7
        self.CRITICAL_SLOWDOWN = 1.5   # Slow down when front < this
        self.EXTREMELY_CLOSE = 1.0     # High gain and evasion threshold
        self.DEADBAND_THRESHOLD = 0.05
        self.danger_close = 0.4        # Emergency reverse threshold
        self.in_danger = False
        self.direction = True
        self.last_steering_angle = 0.0

        # Follow-left mode by default; 1 = follow right when reversing
        self.ulta = 0

        # --- Dynamic Lidar Parameters ---
        self.FRONT_INDEX = None
        self.LEFT_SIDE_INDEX = None
        self.RIGHT_SIDE_INDEX = None
        self.FRONT_45_INDEX = None
        self.FRONT_NEG45_INDEX = None
        self.FRONT_135_INDEX = None
        self.FRONT_NEG135_INDEX = None

        self.is_config_loaded = False

        self.NINETY_DEGREE_RAD = math.pi / 2.0

    def odom_callback(self, msg: Odometry):
        """Placeholder for Odometry, currently unused in core logic."""
        pass

    # 🧮 Dynamic Index Calculation
    def calculate_90_degree_indices(self, msg: LaserScan):
        """Calculates the 0, +90, and -90 degree indices based on scan arithmetic."""

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)
        

        def angle_to_index(target_angle):
            index_float = (target_angle - angle_min) / angle_increment
            index = max(0, min(num_readings - 1, int(index_float)))
            return index

        self.FRONT_INDEX = angle_to_index(0.0) 
        self.LEFT_SIDE_INDEX = angle_to_index(self.NINETY_DEGREE_RAD)
        self.RIGHT_SIDE_INDEX = angle_to_index(-self.NINETY_DEGREE_RAD)
        self.FRONT_45_INDEX = angle_to_index(math.pi / 4.0)
        self.FRONT_NEG45_INDEX = angle_to_index(-math.pi / 4.0)  
        self.FRONT_135_INDEX = angle_to_index(3 * math.pi / 4.0)
        self.FRONT_NEG135_INDEX = angle_to_index(-3 * math.pi / 4.0)

        self.get_logger().info(
            f"Lidar Config Loaded: Front Index={self.FRONT_INDEX}, "
            f"Left Index={self.LEFT_SIDE_INDEX}, Right Index={self.RIGHT_SIDE_INDEX}"
        )
        self.is_config_loaded = True

    def get_indexed_range(self, ranges, index):
        """Helper to get a range reading based on a specific index, using 5-point smoothing."""
        num_readings = len(ranges)

        if index is None or index < 0 or index >= num_readings:
            return 100.0

        start = max(0, index - 10)  #earlier 2
        end = min(num_readings, index + 10) #earlier 3

        segment = ranges[start:end]
        valid_readings = [r for r in segment if r > 0.01 and not math.isinf(r)]

        if not valid_readings:
            return 100.0

        return sum(valid_readings) / len(valid_readings)

    def publish_drive_command(self, speed, steering_angle):
        """Helper to package and publish the drive command."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering_angle)
        self.drive_publisher.publish(drive_msg)
        self.get_logger().debug(f'Published: Speed={speed:.2f}, Steer={steering_angle:.2f}')
        self.last_steering_angle = steering_angle

    def scan_callback(self, msg: LaserScan):
        """Core P-control wall following logic with dynamic safety."""

        if not self.is_config_loaded:
            self.calculate_90_degree_indices(msg)
            if not self.is_config_loaded:
                return

        ranges = msg.ranges

        # 1. Perception
        front_distance = self.get_indexed_range(ranges, self.FRONT_INDEX)
        right_distance = self.get_indexed_range(ranges, self.RIGHT_SIDE_INDEX)
        left_distance = self.get_indexed_range(ranges, self.LEFT_SIDE_INDEX)
        front_45_distance = self.get_indexed_range(ranges, self.FRONT_45_INDEX)
        front_neg45_distance = self.get_indexed_range(ranges, self.FRONT_NEG45_INDEX)
        front_neg135_distance = self.get_indexed_range(ranges, self.FRONT_NEG135_INDEX)
        front_135_distance = self.get_indexed_range(ranges, self.FRONT_135_INDEX)


        self.get_logger().info(
            f"Front={front_distance:.2f}m | LEFT (90)={left_distance:.2f}m | RIGHT (-90)={right_distance:.2f}m"
        )

        # --- Initialize command variables ---
        speed = self.MAX_SPEED
        steering_angle = 0.0
        wall_distance = left_distance  # Following left wall
        error = 0.0

        # ====================================================================
        # 2. Safety Check & Dynamic Evasion 
        # ====================================================================
        
        is_left_close = left_distance < self.EXTREMELY_CLOSE
        is_right_close = right_distance < self.EXTREMELY_CLOSE
        is_side_blocked = is_left_close or is_right_close
        is_hitting = (
            (left_distance < self.danger_close and front_neg45_distance < self.danger_close) or
            (front_45_distance < self.danger_close and right_distance < self.danger_close) or
            front_distance < self.danger_close
        )

        if is_hitting:
            self.get_logger().error(
                f"!! Danger Ahead !! "
                #f"Front = {front_distance:.2f}, Left = {left_distance:.2f}, Right = {right_distance:.2f}"
            )

            # Switch to right-wall following when hit
            
            self.in_danger = True
            
        
        if self.in_danger :
            # After reversing, reset to left-wall following
            speed = -self.MIN_SPEED  # reverse
            
            steering_angle = self.last_steering_angle
            self.get_logger().info("Reversing")
            self.publish_drive_command(speed, steering_angle)
            
            if front_distance > self.CRITICAL_SLOWDOWN or front_neg135_distance < self.danger_close:
                self.in_danger = False  # safe now
                self.direction = False
                self.ulta = 1 - self.ulta  # toggle wall following side
            return

        # Condition 1: Check if front obstacle is within CRITICAL_SLOWDOWN
        if front_distance < self.CRITICAL_SLOWDOWN:
                    
            # Condition 2: If front is close AND a side is close, initiate evasion
            if is_side_blocked:
                speed = self.MIN_SPEED
                self.get_logger().error(
                    f"EVASION: Front < {self.CRITICAL_SLOWDOWN}m AND Side < {self.EXTREMELY_CLOSE}m."
                )

                if left_distance > right_distance:
                    steering_angle = self.MAX_STEER
                    self.get_logger().error(
                        f"Evasion: Turning LEFT (L={left_distance:.2f}m, R={right_distance:.2f}m)"
                    )
                else:
                    steering_angle = -self.MAX_STEER
                    self.get_logger().error(
                        f"Evasion: Turning RIGHT (L={left_distance:.2f}m, R={right_distance:.2f}m)"
                    )

                self.publish_drive_command(speed, steering_angle)
                return

            else:
                speed = self.MIN_SPEED
                self.get_logger().warn(
                    f"CAUTION: Obstacle ahead at {front_distance:.2f}m. Slowing down."
                )

        # ====================================================================
        # 3. Wall Following (P-Control)
        # ====================================================================

        current_gain = self.STEERING_GAIN
        if front_distance < self.EXTREMELY_CLOSE:
            current_gain *= 2.0
            self.get_logger().warn(f"P-Gain Doubled (Front={front_distance:.2f}m).")

        

        # ---- LEFT FOLLOWING MODE ----
        if self.ulta == 0:
            error = self.TARGET_DISTANCE - left_distance
            self.get_logger().warn(f"Pcontrol is ON - left following (Gain={current_gain:.2f})")
            if abs(error) > self.DEADBAND_THRESHOLD:
                steering_angle = -current_gain * error
                steering_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steering_angle))
            else:
                steering_angle = 0.0

        # ---- RIGHT FOLLOWING MODE ----
        else:
            error = self.TARGET_DISTANCE - right_distance
            self.get_logger().warn(f"Pcontrol is ON - right following (Gain={current_gain:.2f})")
            if abs(error) > self.DEADBAND_THRESHOLD:
                steering_angle = current_gain * error
                steering_angle = max(-self.MAX_STEER, min(self.MAX_STEER, steering_angle))
            else:
                steering_angle = 0.0

        # Speed reduction for sharp turns
        if abs(steering_angle) > 0.15:
            speed = self.MIN_SPEED
        elif front_distance >= self.CRITICAL_SLOWDOWN:
            speed = self.MAX_SPEED

        self.publish_drive_command(speed, steering_angle)


def main(args=None):
    rclpy.init(args=args)
    agent_node = WallFollowerAgent()
    agent_node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        pass

    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
