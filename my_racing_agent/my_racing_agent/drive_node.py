import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class WallFollowerAgent(Node):
    """
    Robust Wall Follower with stuck detection, timeout mechanisms, and recovery behaviors.
    """
    def __init__(self):
        super().__init__('wall_follower_agent')
        self.get_logger().info('Robust Wall Follower Agent Initialized.')

        # --- Publishers and Subscribers ---
        self.drive_publisher = self.create_publisher(Twist, '/car_1/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/car_1/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/car_1/ground_truth', self.odom_callback, 10)

        # --- Controller Parameters ---
        self.MAX_SPEED = 2.0
        self.MIN_SPEED = 1.0
        self.REVERSE_SPEED = 1.2  # Increased for better backing out
        self.TARGET_DISTANCE = 1.5
        self.STEERING_GAIN = 0.7
        self.MAX_STEER = 0.7  # Increased for sharper turns
        self.CRITICAL_SLOWDOWN = 1.5
        self.EXTREMELY_CLOSE = 1.0
        self.DEADBAND_THRESHOLD = 0.05
        self.danger_close = 0.5  # Slightly increased threshold

        # --- State Variables ---
        self.in_danger = False
        self.is_stuck = False
        self.ulta = 0  # 0 = follow left, 1 = follow right
        self.last_steering_angle = 0.0

        # --- Stuck Detection ---
        self.current_position = None
        self.last_position = None
        self.position_history = []  # Store last N positions
        self.stuck_check_interval = 2.0  # Check every 2 seconds
        self.last_stuck_check_time = time.time()
        self.stuck_threshold = 0.5  # If moved less than 0.5m in 2 seconds
        self.current_velocity = 0.0
        self.velocity_threshold = 0.5  # Consider stuck if velocity < this

        # --- Recovery Timeouts ---
        self.reverse_start_time = None
        self.max_reverse_duration = 2.0  # Reduced to 2 seconds (faster timeout)
        self.recovery_start_time = None
        self.max_recovery_duration = 5.0
        self.stuck_recovery_counter = 0
        self.max_stuck_attempts = 3

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

        # --- Watchdog Timer ---
        self.last_scan_time = time.time()
        self.scan_timeout = 1.0  # If no scan for 1 second, stop

    def odom_callback(self, msg: Odometry):
        """Track position and velocity for stuck detection."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        # Calculate velocity magnitude
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_velocity = math.sqrt(vx**2 + vy**2)

    def calculate_90_degree_indices(self, msg: LaserScan):
        """Calculates lidar indices for key angles."""
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)

        def angle_to_index(target_angle):
            index_float = (target_angle - angle_min) / angle_increment
            return max(0, min(num_readings - 1, int(index_float)))

        self.FRONT_INDEX = angle_to_index(0.0)
        self.LEFT_SIDE_INDEX = angle_to_index(self.NINETY_DEGREE_RAD)
        self.RIGHT_SIDE_INDEX = angle_to_index(-self.NINETY_DEGREE_RAD)
        self.FRONT_45_INDEX = angle_to_index(math.pi / 4.0)
        self.FRONT_NEG45_INDEX = angle_to_index(-math.pi / 4.0)
        self.FRONT_135_INDEX = angle_to_index(3 * math.pi / 4.0)
        self.FRONT_NEG135_INDEX = angle_to_index(-3 * math.pi / 4.0)

        self.get_logger().info(
            f"Lidar Configured: Front={self.FRONT_INDEX}, "
            f"Left={self.LEFT_SIDE_INDEX}, Right={self.RIGHT_SIDE_INDEX}"
        )
        self.is_config_loaded = True

    def get_indexed_range(self, ranges, index):
        """Get smoothed range reading at specific index."""
        num_readings = len(ranges)
        if index is None or index < 0 or index >= num_readings:
            return 100.0

        start = max(0, index - 10)
        end = min(num_readings, index + 11)
        segment = ranges[start:end]
        valid = [r for r in segment if 0.01 < r < 100.0 and not math.isinf(r)]

        return sum(valid) / len(valid) if valid else 100.0

    def check_if_stuck(self):
        """Detect if the car is stuck and not making progress."""
        current_time = time.time()
        
        # Check 1: Velocity-based detection
        if self.current_velocity < self.velocity_threshold and not self.in_danger:
            if not self.is_stuck:
                self.get_logger().warn(f"LOW VELOCITY DETECTED: {self.current_velocity:.3f} m/s")
                self.is_stuck = True
                self.recovery_start_time = current_time
                self.stuck_recovery_counter += 1
            return True

        # Check 2: Position-based detection (every 2 seconds)
        if current_time - self.last_stuck_check_time >= self.stuck_check_interval:
            if self.current_position and self.last_position:
                dx = self.current_position[0] - self.last_position[0]
                dy = self.current_position[1] - self.last_position[1]
                distance_moved = math.sqrt(dx**2 + dy**2)

                if distance_moved < self.stuck_threshold:
                    self.get_logger().warn(
                        f"STUCK DETECTED: Moved only {distance_moved:.3f}m in {self.stuck_check_interval}s"
                    )
                    self.is_stuck = True
                    self.recovery_start_time = current_time
                    self.stuck_recovery_counter += 1
                    return True

            self.last_position = self.current_position
            self.last_stuck_check_time = current_time

        return False

    def execute_recovery_behavior(self, front_distance, left_distance, right_distance):
        """Execute recovery behavior when stuck."""
        current_time = time.time()

        # Timeout recovery mode after max duration
        if self.recovery_start_time and (current_time - self.recovery_start_time) > self.max_recovery_duration:
            self.get_logger().info("Recovery timeout reached, resuming normal operation")
            self.is_stuck = False
            self.recovery_start_time = None
            return None

        # Recovery strategy based on attempt count
        if self.stuck_recovery_counter % 3 == 1:
            # Strategy 1: Reverse and turn sharply
            self.get_logger().warn("RECOVERY 1: Reverse + Sharp Turn")
            speed = -self.REVERSE_SPEED
            steering = self.MAX_STEER if left_distance > right_distance else -self.MAX_STEER
            return (speed, steering)
        
        elif self.stuck_recovery_counter % 3 == 2:
            # Strategy 2: Turn in place
            self.get_logger().warn("RECOVERY 2: Turn in Place")
            speed = 0.0
            steering = -self.MAX_STEER if self.ulta == 0 else self.MAX_STEER
            return (speed, steering)
        
        else:
            # Strategy 3: Switch wall following side
            self.get_logger().warn("RECOVERY 3: Switch Wall Following Side")
            self.ulta = 1 - self.ulta
            speed = self.MIN_SPEED
            steering = self.MAX_STEER if self.ulta == 0 else -self.MAX_STEER
            return (speed, steering)

    def publish_drive_command(self, speed, steering_angle):
        """Publish drive command with safety limits."""
        cmd = Twist()
        cmd.linear.x = float(max(-self.MAX_SPEED, min(self.MAX_SPEED, speed)))
        cmd.angular.z = float(max(-self.MAX_STEER, min(self.MAX_STEER, steering_angle)))
        self.drive_publisher.publish(cmd)
        self.last_steering_angle = steering_angle

    def scan_callback(self, msg: LaserScan):
        """Main control loop with robust stuck handling."""
        self.last_scan_time = time.time()

        if not self.is_config_loaded:
            self.calculate_90_degree_indices(msg)
            if not self.is_config_loaded:
                return

        ranges = msg.ranges

        # Get sensor readings
        front_distance = self.get_indexed_range(ranges, self.FRONT_INDEX)
        left_distance = self.get_indexed_range(ranges, self.LEFT_SIDE_INDEX)
        right_distance = self.get_indexed_range(ranges, self.RIGHT_SIDE_INDEX)
        front_45 = self.get_indexed_range(ranges, self.FRONT_45_INDEX)
        front_neg45 = self.get_indexed_range(ranges, self.FRONT_NEG45_INDEX)
        front_135 = self.get_indexed_range(ranges, self.FRONT_135_INDEX)
        front_neg135 = self.get_indexed_range(ranges, self.FRONT_NEG135_INDEX)

        self.get_logger().info(
            f"F={front_distance:.2f} | L={left_distance:.2f} | R={right_distance:.2f} | "
            f"Vel={self.current_velocity:.2f} | Stuck={self.is_stuck}"
        )

        # ============================================================
        # PRIORITY 1: Check if stuck and execute recovery
        # ============================================================
        if self.check_if_stuck():
            recovery_cmd = self.execute_recovery_behavior(front_distance, left_distance, right_distance)
            if recovery_cmd:
                speed, steering = recovery_cmd
                self.publish_drive_command(speed, steering)
                return

        # If velocity recovered, clear stuck flag
        if self.current_velocity > self.velocity_threshold * 2:
            if self.is_stuck:
                self.get_logger().info("RECOVERED: Vehicle moving again")
            self.is_stuck = False
            self.recovery_start_time = None

        # ============================================================
        # PRIORITY 2: Emergency collision avoidance
        # ============================================================
        is_hitting = (
            front_distance < self.danger_close or
            (left_distance < self.danger_close and front_neg45 < self.danger_close) or
            (right_distance < self.danger_close and front_45 < self.danger_close)
        )

        if is_hitting:
            if not self.in_danger:
                self.get_logger().error("COLLISION IMMINENT!")
                self.in_danger = True
                self.reverse_start_time = time.time()

        if self.in_danger:
            current_time = time.time()
            
            # Check reverse timeout
            if self.reverse_start_time and (current_time - self.reverse_start_time) > self.max_reverse_duration:
                self.get_logger().warn("Reverse timeout reached, forcing exit from danger mode")
                self.in_danger = False
                self.reverse_start_time = None
                self.ulta = 1 - self.ulta  # Switch sides
                # Force forward movement with sharp turn
                speed = self.MIN_SPEED
                steering = self.MAX_STEER if left_distance > right_distance else -self.MAX_STEER
                self.publish_drive_command(speed, steering)
                return

            # AGGRESSIVE reverse maneuver with SHARP turning
            speed = -self.REVERSE_SPEED * 1.5  # Faster reverse
            
            # Determine turn direction based on available space
            # Priority: Turn toward the most open space
            if left_distance > right_distance + 0.2:
                # More space on left, turn hard right while reversing
                steering = self.MAX_STEER
                self.get_logger().warn(f"REVERSING RIGHT (L={left_distance:.2f} > R={right_distance:.2f})")
            elif right_distance > left_distance + 0.2:
                # More space on right, turn hard left while reversing  
                steering = -self.MAX_STEER
                self.get_logger().warn(f"REVERSING LEFT (R={right_distance:.2f} > L={left_distance:.2f})")
            else:
                # Equal space, alternate based on time
                if int(current_time * 2) % 2 == 0:
                    steering = self.MAX_STEER
                else:
                    steering = -self.MAX_STEER
                self.get_logger().warn("REVERSING with alternating turn")
            
            self.publish_drive_command(speed, steering)

            # RELAXED exit condition: just need front clearance
            if front_distance > self.danger_close * 1.5:
                self.get_logger().info(f"Front cleared to {front_distance:.2f}m, exiting danger mode")
                self.in_danger = False
                self.reverse_start_time = None
                self.ulta = 1 - self.ulta  # Switch wall following side
            return

        # ============================================================
        # PRIORITY 3: Evasive maneuvering (tight spaces)
        # ============================================================
        is_tight = front_distance < self.CRITICAL_SLOWDOWN
        is_left_blocked = left_distance < self.EXTREMELY_CLOSE
        is_right_blocked = right_distance < self.EXTREMELY_CLOSE

        if is_tight and (is_left_blocked or is_right_blocked):
            speed = self.MIN_SPEED
            # Turn toward more open space
            if left_distance > right_distance + 0.3:  # Significant difference
                steering = self.MAX_STEER
                self.get_logger().warn(f"TIGHT SPACE: Turning LEFT (L={left_distance:.2f})")
            elif right_distance > left_distance + 0.3:
                steering = -self.MAX_STEER
                self.get_logger().warn(f"TIGHT SPACE: Turning RIGHT (R={right_distance:.2f})")
            else:
                # Equal distances, make a decision based on current following side
                steering = -self.MAX_STEER if self.ulta == 0 else self.MAX_STEER
                self.get_logger().warn("TIGHT SPACE: Equal clearance, turning based on mode")
            
            self.publish_drive_command(speed, steering)
            return

        # ============================================================
        # PRIORITY 4: Normal wall following (P-control)
        # ============================================================
        speed = self.MAX_SPEED
        current_gain = self.STEERING_GAIN

        # Increase gain when approaching obstacles
        if front_distance < self.EXTREMELY_CLOSE:
            current_gain *= 1.5

        # LEFT WALL FOLLOWING
        if self.ulta == 0:
            error = self.TARGET_DISTANCE - left_distance
            if abs(error) > self.DEADBAND_THRESHOLD:
                steering = -current_gain * error
            else:
                steering = 0.0

        # RIGHT WALL FOLLOWING
        else:
            error = self.TARGET_DISTANCE - right_distance
            if abs(error) > self.DEADBAND_THRESHOLD:
                steering = current_gain * error
            else:
                steering = 0.0

        # Reduce speed for sharp turns
        if abs(steering) > 0.2:
            speed = self.MIN_SPEED
        elif front_distance < self.CRITICAL_SLOWDOWN:
            speed = self.MIN_SPEED

        self.publish_drive_command(speed, steering)


def main(args=None):
    rclpy.init(args=args)
    agent_node = WallFollowerAgent()
    agent_node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        pass

    # Stop the car before shutting down
    stop_cmd = Twist()
    agent_node.drive_publisher.publish(stop_cmd)
    
    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()