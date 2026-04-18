#!/usr/bin/env python3

import csv
import json
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
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


class BCTrajectoryFollower(Node):
    def __init__(self) -> None:
        super().__init__("bc_trajectory_follower")

        self.declare_parameter("model_file", "")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "car_1_base_link")
        self.declare_parameter("scan_topic", "/car_1/scan")
        self.declare_parameter("odom_topic", "/car_1/odom")
        self.declare_parameter("cmd_topic", "/car_1/cmd_vel")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("lookahead_distance", 0.8)
        self.declare_parameter("tf_warmup_count", 5)

        self.model_file = str(self.get_parameter("model_file").value)
        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.tf_warmup_count = int(self.get_parameter("tf_warmup_count").value)

        if not self.model_file:
            raise ValueError("model_file parameter is required")

        with open(self.model_file, "r", encoding="utf-8") as f:
            model = json.load(f)

        self.feature_names: List[str] = model["feature_names"]
        self.w = np.asarray(model["weights"], dtype=np.float64)  # [features, 2]
        self.b = np.asarray(model["bias"], dtype=np.float64)     # [2]
        self.x_mean = np.asarray(model["x_mean"], dtype=np.float64)
        self.x_std = np.asarray(model["x_std"], dtype=np.float64)
        self.y_mean = np.asarray(model["y_mean"], dtype=np.float64)
        self.y_std = np.asarray(model["y_std"], dtype=np.float64)

        self.trajectory: List[Tuple[float, float, float, float]] = []
        if self.trajectory_file:
            self.trajectory = self._load_trajectory(self.trajectory_file)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.current_target_idx = 0
        self.finished = False
        self._tf_warmup_remaining = self.tf_warmup_count

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.control_rate, 1.0), self._control_step)

        self.get_logger().info(f"Loaded BC model: {self.model_file}")
        if self.trajectory:
            self.get_logger().info(f"Loaded trajectory: {self.trajectory_file} ({len(self.trajectory)} points)")
        else:
            self.get_logger().warn("No trajectory_file provided to BC follower; trajectory-based features may be unavailable")

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

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

    def _stop_robot(self) -> None:
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}", throttle_duration_sec=2.0)
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return tx, ty, yaw

    def _scan_stats(self) -> Dict[str, float]:
        out = {
            "scan_count": math.nan,
            "scan_min": math.nan,
            "scan_max": math.nan,
            "scan_mean": math.nan,
            "scan_std": math.nan,
            "scan_front_min": math.nan,
            "scan_left_min": math.nan,
            "scan_right_min": math.nan,
        }
        if self.latest_scan is None or not self.latest_scan.ranges:
            return out

        ranges = [r for r in self.latest_scan.ranges if math.isfinite(r)]
        if not ranges:
            return out

        n = len(ranges)
        mean = sum(ranges) / n
        var = sum((v - mean) * (v - mean) for v in ranges) / n
        std = math.sqrt(var)

        raw = [r if math.isfinite(r) else math.nan for r in self.latest_scan.ranges]
        total = len(raw)
        if total >= 9:
            third = total // 3
            right = [v for v in raw[:third] if math.isfinite(v)]
            front = [v for v in raw[third:2 * third] if math.isfinite(v)]
            left = [v for v in raw[2 * third:] if math.isfinite(v)]
            front_min = min(front) if front else math.nan
            left_min = min(left) if left else math.nan
            right_min = min(right) if right else math.nan
        else:
            front_min = left_min = right_min = min(ranges)

        out.update(
            {
                "scan_count": float(len(self.latest_scan.ranges)),
                "scan_min": min(ranges),
                "scan_max": max(ranges),
                "scan_mean": mean,
                "scan_std": std,
                "scan_front_min": front_min,
                "scan_left_min": left_min,
                "scan_right_min": right_min,
            }
        )
        return out

    def _trajectory_metrics(self, pose: Optional[Tuple[float, float, float]]) -> Dict[str, float]:
        out = {
            "traj_nearest_idx": math.nan,
            "traj_target_idx": math.nan,
            "traj_cross_track_error": math.nan,
            "traj_heading_error": math.nan,
            "traj_goal_distance": math.nan,
        }
        if pose is None or not self.trajectory:
            return out

        rx, ry, ryaw = pose
        nearest_idx = self.current_target_idx
        nearest_dist = float("inf")

        search_start = 0 if self.current_target_idx == 0 else self.current_target_idx
        search_end = len(self.trajectory) if self.current_target_idx == 0 else min(self.current_target_idx + 400, len(self.trajectory))

        for i in range(search_start, search_end):
            tx, ty, _, _ = self.trajectory[i]
            d = math.hypot(tx - rx, ty - ry)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        target_idx = nearest_idx
        for i in range(nearest_idx, len(self.trajectory)):
            tx, ty, _, _ = self.trajectory[i]
            if math.hypot(tx - rx, ty - ry) >= self.lookahead_distance:
                target_idx = i
                break
            target_idx = i

        self.current_target_idx = max(self.current_target_idx, target_idx)

        tx, ty, tyaw, _ = self.trajectory[self.current_target_idx]
        dx = tx - rx
        dy = ty - ry
        cte = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        heading_error = normalize_angle(tyaw - ryaw)

        gx, gy, _, _ = self.trajectory[-1]
        goal_dist = math.hypot(gx - rx, gy - ry)

        out.update(
            {
                "traj_nearest_idx": float(nearest_idx),
                "traj_target_idx": float(self.current_target_idx),
                "traj_cross_track_error": cte,
                "traj_heading_error": heading_error,
                "traj_goal_distance": goal_dist,
            }
        )
        return out

    def _feature_dict(self, pose: Optional[Tuple[float, float, float]]) -> Dict[str, float]:
        feat: Dict[str, float] = {}

        if self.latest_odom is None:
            feat["odom_linear_x"] = math.nan
            feat["odom_angular_z"] = math.nan
        else:
            feat["odom_linear_x"] = self.latest_odom.twist.twist.linear.x
            feat["odom_angular_z"] = self.latest_odom.twist.twist.angular.z

        feat.update(self._scan_stats())
        feat.update(self._trajectory_metrics(pose))
        return feat

    def _predict_cmd(self, x_raw: np.ndarray) -> Tuple[float, float]:
        x_n = (x_raw - self.x_mean) / self.x_std
        y_n = x_n @ self.w + self.b
        y = y_n * self.y_std + self.y_mean
        return float(y[0]), float(y[1])

    def _control_step(self) -> None:
        if self.finished:
            self._stop_robot()
            return

        pose = self._lookup_pose()
        if pose is None:
            self._stop_robot()
            return

        if self._tf_warmup_remaining > 0:
            self._tf_warmup_remaining -= 1
            self._stop_robot()
            return

        feat = self._feature_dict(pose)
        x_vals: List[float] = []
        for name in self.feature_names:
            val = feat.get(name, math.nan)
            if not math.isfinite(val):
                self.get_logger().warn(f"Missing/invalid feature '{name}', holding command", throttle_duration_sec=2.0)
                self._stop_robot()
                return
            x_vals.append(val)

        if self.trajectory:
            goal_dist = feat.get("traj_goal_distance", math.nan)
            if math.isfinite(goal_dist) and goal_dist <= self.goal_tolerance:
                self.finished = True
                self._stop_robot()
                self.get_logger().info("BC follower goal reached. Stopping.")
                return

        v_cmd, w_cmd = self._predict_cmd(np.asarray(x_vals, dtype=np.float64))

        v_cmd = max(0.0, min(v_cmd, self.max_linear_speed))
        w_cmd = max(-self.max_angular_speed, min(w_cmd, self.max_angular_speed))

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BCTrajectoryFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._stop_robot()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
