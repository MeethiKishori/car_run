import rclpy
import sys
import numpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import torch
import torch.nn as nn


import pickle
import math
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class WallFollowerNet(nn.Module):
    """Neural network for wall following control"""
    def __init__(self, input_size=4, hidden_sizes=[128, 64, 32], output_size=2, dropout=0.2):
        super(WallFollowerNet, self).__init__()
        
        layers = []
        prev_size = input_size
        
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(prev_size, hidden_size))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(dropout))
            prev_size = hidden_size
        
        layers.append(nn.Linear(prev_size, output_size))
        
        self.network = nn.Sequential(*layers)
    
    def forward(self, x):
        return self.network(x)

class NNWallFollower(Node):
    """
    Neural Network Wall Follower Controller
    Uses trained model to predict control commands
    """
    def __init__(self):
        super().__init__('nn_wall_follower')
        self.get_logger().info('Neural Network Wall Follower Initialized')

        # Publishers and Subscribers
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Load model and scaler
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.load_model()
        
        # Control Parameters (safety overrides)
        self.TARGET_DISTANCE = 1.0
        self.MAX_SPEED = 2.0
        self.MIN_SPEED = 0.5
        self.MAX_ANGULAR = 0.6
        
        # Collision thresholds
        self.FRONT_DANGER = 0.6
        self.SIDE_DANGER = 0.4
        
        # Wall following
        self.follow_left = True
        
        # Lidar indices
        self.FRONT_INDEX = None
        self.LEFT_INDEX = None
        self.RIGHT_INDEX = None
        self.is_config_loaded = False
        
        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def load_model(self):
        """Load trained model, scaler, and config"""
        
        # Get package share directory
        pkg_dir = get_package_share_directory('my_racing_agent')
        models_dir = Path(pkg_dir) / 'models'

        model_path = models_dir / 'wall_follower_model.pth'
        scaler_path = models_dir / 'scaler.pkl'
        config_path = models_dir / 'model_config.json'  # if you use config

        # Check files exist
        if not model_path.exists():
            self.get_logger().error(f'Model file not found: {model_path}')
            raise FileNotFoundError(f'Model file not found: {model_path}')
        
        if not scaler_path.exists():
            self.get_logger().error(f'Scaler file not found: {scaler_path}')
            raise FileNotFoundError(f'Scaler file not found: {scaler_path}')
        
        if not config_path.exists():
            self.get_logger().warn(f'Config file not found: {config_path}')

        # Load model
        self.model = WallFollowerNet(input_size=4, hidden_sizes=[128, 64, 32], output_size=2)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        # Load scaler
        with open(scaler_path, 'rb') as f:
            self.scaler = pickle.load(f)

        self.get_logger().info(f'Model and scaler loaded successfully from {models_dir} on {self.device}')

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Update velocities
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z

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

    def predict_control(self, cte):
        """Use neural network to predict control commands"""
        # Prepare features: [cte, theta, linear_speed, angular_speed]
        features = np.array([[cte, self.theta, self.linear_speed, self.angular_speed]])
        
        # Normalize
        features_scaled = self.scaler.transform(features)
        
        # Convert to tensor
        features_tensor = torch.FloatTensor(features_scaled).to(self.device)
        
        # Predict
        with torch.no_grad():
            outputs = self.model(features_tensor)
        
        # Extract predictions
        predictions = outputs.cpu().numpy()[0]
        speed_cmd = predictions[0]
        angular_cmd = predictions[1]
        
        return speed_cmd, angular_cmd

    def scan_callback(self, msg: LaserScan):
        if not self.is_config_loaded:
            self.calculate_lidar_indices(msg)
            return

        ranges = msg.ranges
        front = self.get_range(ranges, self.FRONT_INDEX)
        left = self.get_range(ranges, self.LEFT_INDEX)
        right = self.get_range(ranges, self.RIGHT_INDEX)

        cmd = Twist()

        # Safety override: Front collision
        if front < self.FRONT_DANGER:
            cmd.linear.x = -self.MIN_SPEED
            cmd.angular.z = self.MAX_ANGULAR if left > right else -self.MAX_ANGULAR
            self.drive_publisher.publish(cmd)
            self.get_logger().warn('SAFETY: Front collision avoidance!')
            return

        # Safety override: Side collision
        if (self.follow_left and left < self.SIDE_DANGER) or \
           (not self.follow_left and right < self.SIDE_DANGER):
            cmd.linear.x = self.MIN_SPEED
            cmd.angular.z = -self.MAX_ANGULAR if self.follow_left else self.MAX_ANGULAR
            self.drive_publisher.publish(cmd)
            self.get_logger().warn('SAFETY: Side collision avoidance!')
            return

        # Compute CTE
        if self.follow_left:
            cte = self.TARGET_DISTANCE - left
        else:
            cte = right - self.TARGET_DISTANCE

        # Get predictions from neural network
        speed_cmd, angular_cmd = self.predict_control(cte)
        
        # Apply safety limits
        speed_cmd = max(self.MIN_SPEED, min(self.MAX_SPEED, speed_cmd))
        angular_cmd = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_cmd))
        
        # Publish command
        cmd.linear.x = float(speed_cmd)
        cmd.angular.z = float(angular_cmd)
        self.drive_publisher.publish(cmd)
        
        self.get_logger().info(
            f'CTE={cte:.3f}, Speed={speed_cmd:.3f}, Angular={angular_cmd:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = NNWallFollower()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            stop = Twist()
            node.drive_publisher.publish(stop)
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
def load_model(self):
        Load trained model and scaler

        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'wall_follower_model.pth')
        scaler_path = os.path.join(script_dir, 'scaler.pkl')
        config_path = os.path.join(script_dir, 'model_config.json')
        
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            raise FileNotFoundError(f'Model file not found: {model_path}')
        
        if not os.path.exists(scaler_path):
            self.get_logger().error(f'Scaler file not found: {scaler_path}')
            raise FileNotFoundError(f'Scaler file not found: {scaler_path}')
        
        # Load model
        self.model = WallFollowerNet(input_size=4, hidden_sizes=[128, 64, 32], output_size=2)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device, weights_only=False))
        self.model.to(self.device)
        self.model.eval()
        
        # Load scaler
        with open(scaler_path, 'rb') as f:
            self.scaler = pickle.load(f)
        
        self.get_logger().info(f'Model and scaler loaded successfully on {self.device}')
"""