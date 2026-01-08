import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class PIDWallFollower(Node):
    """
    Clean PID Wall Follower Controller
    - Uncomment the controller you want to test: P, PI, or PID
    """
    def __init__(self):
        super().__init__('pid_wall_follower')
        self.get_logger().info('PID Wall Follower Initialized')

        # Publishers and Subscribers
        #self.drive_publisher = self.create_publisher(Twist, '/car_1/cmd_vel', 10)
        #self.scan_subscriber = self.create_subscription(
        #    LaserScan, '/car_1/scan', self.scan_callback, 10, no to just /scan & /cmd_vel
        #)
        # With the f1tenth_gym topic names:
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # ============================================================
        # CONTROLLER SELECTION - Comment/Uncomment to choose
        # ============================================================
        self.controller_mode = 'P'      # P-Control only
        #self.controller_mode = 'PI'   #  PI-Control
        #self.controller_mode = 'PID'  # ✅ Active:  PID-Control

        # ============================================================
        # PID GAINS - Tune these values
        # ============================================================
        self.Kp = 0.8          # Proportional gain
        self.Ki = 0.1         # Integral gain (for PI/PID)
        self.Kd = 0.5          # Derivative gain (for PID)

        # ============================================================
        # CONTROL PARAMETERS
        # ============================================================
        self.TARGET_DISTANCE = 1.0      # Desired distance from wall (meters)
        self.MAX_SPEED = 2.0            # Maximum forward speed
        self.MIN_SPEED = 0.5            # Minimum speed (for safety/corners)
        self.MAX_ANGULAR = 0.6          # Maximum turning rate (rad/s)
        
        # Collision avoidance thresholds
        self.FRONT_DANGER = 0.6         # Stop/reverse if front < this
        self.SIDE_DANGER = 0.4          # Emergency if side < this
        
        # Wall following mode
        self.follow_left = True         # True = follow left wall, False = follow right

        # ============================================================
        # PID STATE VARIABLES
        # ============================================================
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Integral anti-windup limits
        self.integral_max = 1.0
        self.integral_min = -1.0

        # ============================================================
        # LIDAR INDICES (calculated dynamically)
        # ============================================================
        self.FRONT_INDEX = None
        self.LEFT_INDEX = None
        self.RIGHT_INDEX = None
        self.is_config_loaded = False

    def calculate_lidar_indices(self, msg: LaserScan):
        """Calculate lidar array indices for 0°, +90°, -90°"""
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
            f'Lidar configured: Front={self.FRONT_INDEX}, '
            f'Left={self.LEFT_INDEX}, Right={self.RIGHT_INDEX}'
        )
        self.is_config_loaded = True

    def get_range(self, ranges, index, window=10):
        """Get smoothed range reading at index with averaging window"""
        if index is None:
            return 100.0
        
        num_readings = len(ranges)
        start = max(0, index - window)
        end = min(num_readings, index + window + 1)
        
        segment = ranges[start:end]
        valid = [r for r in segment if 0.1 < r < 30.0 and not math.isinf(r)]
        
        return sum(valid) / len(valid) if valid else 100.0

    def compute_control(self, error, dt):
        """
        Compute control output based on selected mode
        Returns: angular velocity command
        """
        # ============================================================
        # P-CONTROL (Proportional only)
        # ============================================================
        if self.controller_mode == 'P':
            control = self.Kp * error
            self.get_logger().info(
                f'[P] Error={error:.3f}, Control={control:.3f}'
            )
        
        # ============================================================
        # PI-CONTROL (Proportional + Integral)
        # ============================================================
        elif self.controller_mode == 'PI':
            # Update integral
            self.integral += error * dt
            # Anti-windup: clamp integral
            self.integral = max(self.integral_min, min(self.integral_max, self.integral))
            
            # PI control law
            control = self.Kp * error + self.Ki * self.integral
            
            self.get_logger().info(
                f'[PI] Error={error:.3f}, Integral={self.integral:.3f}, Control={control:.3f}'
            )
        
        # ============================================================
        # PID-CONTROL (Proportional + Integral + Derivative)
        # ============================================================
        elif self.controller_mode == 'PID':
            # Update integral
            self.integral += error * dt
            # Anti-windup: clamp integral
            self.integral = max(self.integral_min, min(self.integral_max, self.integral))
            
            # Calculate derivative
            derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
            
            # PID control law
            control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            
            self.get_logger().info(
                f'[PID] Error={error:.3f}, Integral={self.integral:.3f}, '
                f'Derivative={derivative:.3f}, Control={control:.3f}'
            )
            
            # Update previous error for next derivative calculation
            self.previous_error = error
        
        else:
            self.get_logger().error(f'Unknown controller mode: {self.controller_mode}')
            control = 0.0

        # Clamp control output to maximum angular velocity
        control = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, control))
        
        return control

    def scan_callback(self, msg: LaserScan):
        """Main control loop"""
        
        # Initialize lidar indices on first scan
        if not self.is_config_loaded:
            self.calculate_lidar_indices(msg)
            return

        # ============================================================
        # 1. READ SENSORS
        # ============================================================
        ranges = msg.ranges
        front = self.get_range(ranges, self.FRONT_INDEX)
        left = self.get_range(ranges, self.LEFT_INDEX)
        right = self.get_range(ranges, self.RIGHT_INDEX)

        self.get_logger().info(
            f'Sensors: Front={front:.2f}m, Left={left:.2f}m, Right={right:.2f}m'
        )

        # ============================================================
        # 2. SAFETY CHECK - Emergency stop/reverse
        # ============================================================
        cmd = Twist()
        
        # Front collision imminent
        if front < self.FRONT_DANGER:
            self.get_logger().warn(f'FRONT DANGER: {front:.2f}m < {self.FRONT_DANGER}m')
            cmd.linear.x = -self.MIN_SPEED  # Reverse
            cmd.angular.z = self.MAX_ANGULAR if left > right else -self.MAX_ANGULAR
            self.drive_publisher.publish(cmd)
            # Reset integral to avoid windup during emergency
            self.integral = 0.0
            return
        
        # Side collision imminent
        if (self.follow_left and left < self.SIDE_DANGER) or \
           (not self.follow_left and right < self.SIDE_DANGER):
            self.get_logger().warn('SIDE DANGER - Emergency turn')
            cmd.linear.x = self.MIN_SPEED
            cmd.angular.z = -self.MAX_ANGULAR if self.follow_left else self.MAX_ANGULAR
            self.drive_publisher.publish(cmd)
            # Reset integral
            self.integral = 0.0
            return

        # ============================================================
        # 3. CALCULATE ERROR
        # ============================================================
        # Error = desired distance - actual distance
        if self.follow_left:
            error = self.TARGET_DISTANCE - left
            # Negative error = too close to wall (steer right/away)
            # Positive error = too far from wall (steer left/toward)
        else:
            error = right - self.TARGET_DISTANCE
            # For right wall, we flip the sign

        # ============================================================
        # 4. COMPUTE CONTROL (PID)
        # ============================================================
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Compute angular velocity using selected controller
        angular_velocity = self.compute_control(error, dt)

        # ============================================================
        # 5. SPEED CONTROL (based on turning rate)
        # ============================================================
        # Slow down for sharp turns
        turn_ratio = abs(angular_velocity) / self.MAX_ANGULAR
        
        if turn_ratio > 0.5:
            speed = self.MIN_SPEED
        elif turn_ratio > 0.3:
            speed = (self.MAX_SPEED + self.MIN_SPEED) / 2.0
        else:
            speed = self.MAX_SPEED
        
        # Slow down when approaching front obstacles
        if front < 2.0:
            speed = min(speed, self.MIN_SPEED)

        # ============================================================
        # 6. PUBLISH COMMAND
        # ============================================================
        cmd.linear.x = speed
        cmd.angular.z = -angular_velocity if self.follow_left else angular_velocity
        
        self.drive_publisher.publish(cmd)

        self.get_logger().info(
            f'Command: Speed={speed:.2f}, Angular={cmd.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDWallFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Stop the car
    stop = Twist()
    node.drive_publisher.publish(stop)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()