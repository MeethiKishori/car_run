#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime
from typing import List, Optional, Tuple

import rclpy
import rclpy.duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def safe_float(v: float) -> float:
    if math.isfinite(v):
        return float(v)
    return float("nan")


class NavigationDataCollector(Node):
    def __init__(self) -> None:
        super().__init__("navigation_data_collector")

        self.declare_parameter("output_file", "")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "car_1_base_link")
        self.declare_parameter("scan_topic", "/car_1/scan")
        self.declare_parameter("imu_topic", "/car_1/imu")
        self.declare_parameter("odom_topic", "/car_1/odom")
        self.declare_parameter("cmd_topic", "/car_1/cmd_vel")
        self.declare_parameter("log_rate", 20.0)
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.output_file = str(self.get_parameter("output_file").value)
        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.log_rate = float(self.get_parameter("log_rate").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        if not self.output_file:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_file = f"/sim_ws/src/my_robot/datasets/nav_dataset_{ts}.csv"

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        self.trajectory: List[Tuple[float, float, float, float]] = []
        if self.trajectory_file:
            self.trajectory = self._load_trajectory(self.trajectory_file)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd: Optional[Twist] = None

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)

        self.csv_file = open(self.output_file, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "timestamp_sec",
            "odom_linear_x", "odom_angular_z",
            "cmd_linear_x", "cmd_angular_z",
            "scan_min", "scan_mean", "scan_std",
            "scan_front_min", "scan_left_min", "scan_right_min",
            "traj_cross_track_error", "traj_heading_error", "traj_goal_distance",
        ])

        self.timer = self.create_timer(1.0 / max(self.log_rate, 1.0), self._log_step)
        self.get_logger().info(f"Data collection started: {self.output_file}")
        if self.trajectory:
            self.get_logger().info(f"Loaded trajectory for error metrics: {self.trajectory_file}")
        else:
            self.get_logger().info("No trajectory loaded: trajectory error metrics will be NaN")

    def _load_trajectory(self, file_path: str) -> List[Tuple[float, float, float, float]]:
        rows: List[Tuple[float, float, float, float]] = []
        with open(file_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row["x"])
                y = float(row["y"])
                yaw = float(row["yaw"])
                v_ref = float(row.get("v_ref", 0.0))
                rows.append((x, y, yaw, v_ref))
        return rows

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _imu_cb(self, msg: Imu) -> None:
        self.latest_imu = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return tx, ty, yaw

    def _scan_stats(self, scan: Optional[LaserScan]) -> Tuple[float, float, float, float, float, float, float, float]:
        if scan is None or not scan.ranges:
            return (0.0, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)

        values = [r for r in scan.ranges if math.isfinite(r)]
        if not values:
            return (float(len(scan.ranges)), math.nan, math.nan, math.nan, math.nan, math.nan, math.nan, math.nan)

        n = len(values)
        mean = sum(values) / n
        var = sum((v - mean) * (v - mean) for v in values) / n
        std = math.sqrt(var)

        # Sector minima from the finite subset of the full scan array.
        raw = [r if math.isfinite(r) else math.nan for r in scan.ranges]
        total = len(raw)
        if total < 9:
            front_min = min(values)
            left_min = min(values)
            right_min = min(values)
        else:
            third = total // 3
            right = [v for v in raw[:third] if math.isfinite(v)]
            front = [v for v in raw[third:2 * third] if math.isfinite(v)]
            left = [v for v in raw[2 * third:] if math.isfinite(v)]
            front_min = min(front) if front else math.nan
            left_min = min(left) if left else math.nan
            right_min = min(right) if right else math.nan

        return (
            float(len(scan.ranges)),
            min(values),
            max(values),
            mean,
            std,
            front_min,
            left_min,
            right_min,
        )

    def _trajectory_metrics(self, pose: Optional[Tuple[float, float, float]]) -> Tuple[float, float, float, float, float]:
        if pose is None or not self.trajectory:
            return (math.nan, math.nan, math.nan, math.nan, math.nan)

        rx, ry, ryaw = pose
        nearest_idx = 0
        nearest_dist = float("inf")
        for i, (tx, ty, _, _) in enumerate(self.trajectory):
            d = math.hypot(tx - rx, ty - ry)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        target_idx = nearest_idx
        lookahead = 0.8
        for i in range(nearest_idx, len(self.trajectory)):
            tx, ty, _, _ = self.trajectory[i]
            if math.hypot(tx - rx, ty - ry) >= lookahead:
                target_idx = i
                break
            target_idx = i

        tx, ty, tyaw, _ = self.trajectory[target_idx]
        dx = tx - rx
        dy = ty - ry
        cross_track_error = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        heading_error = normalize_angle(tyaw - ryaw)

        gx, gy, _, _ = self.trajectory[-1]
        goal_dist = math.hypot(gx - rx, gy - ry)

        return (
            float(nearest_idx),
            float(target_idx),
            cross_track_error,
            heading_error,
            goal_dist,
        )

    def _log_step(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        pose = self._lookup_pose()
        scan_count, scan_min, scan_max, scan_mean, scan_std, front_min, left_min, right_min = self._scan_stats(self.latest_scan)
        nearest_idx, target_idx, cte, heading_err, goal_dist = self._trajectory_metrics(pose)

        if pose is None:
            pose_x = pose_y = pose_yaw = math.nan
        else:
            pose_x, pose_y, pose_yaw = pose

        if self.latest_odom is None:
            odom_vx = odom_wz = math.nan
        else:
            odom_vx = self.latest_odom.twist.twist.linear.x
            odom_wz = self.latest_odom.twist.twist.angular.z

        if self.latest_cmd is None:
            cmd_vx = cmd_wz = math.nan
        else:
            cmd_vx = self.latest_cmd.linear.x
            cmd_wz = self.latest_cmd.angular.z

        if self.latest_imu is None:
            imu_ax = imu_ay = imu_az = math.nan
            imu_gx = imu_gy = imu_gz = math.nan
        else:
            imu_ax = self.latest_imu.linear_acceleration.x
            imu_ay = self.latest_imu.linear_acceleration.y
            imu_az = self.latest_imu.linear_acceleration.z
            imu_gx = self.latest_imu.angular_velocity.x
            imu_gy = self.latest_imu.angular_velocity.y
            imu_gz = self.latest_imu.angular_velocity.z

        self.writer.writerow([
            safe_float(now),
            safe_float(odom_vx), safe_float(odom_wz),
            safe_float(cmd_vx), safe_float(cmd_wz),
            safe_float(scan_min), safe_float(scan_mean), safe_float(scan_std),
            safe_float(front_min), safe_float(left_min), safe_float(right_min),
            safe_float(cte), safe_float(heading_err), safe_float(goal_dist),
        ])
        self.csv_file.flush()

    def close(self) -> None:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NavigationDataCollector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
