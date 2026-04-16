#!/usr/bin/env python3

import csv
import math
from typing import List, Optional, Tuple

import rclpy
import rclpy.duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


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


class PIDTrajectoryFollower(Node):
    def __init__(self) -> None:
        super().__init__("pid_trajectory_follower")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "car_1_base_link")
        self.declare_parameter("cmd_topic", "/car_1/cmd_vel")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("lookahead_distance", 0.8)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("min_linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("pid_kp", 1.6)
        self.declare_parameter("pid_ki", 0.04)
        self.declare_parameter("pid_kd", 0.18)
        self.declare_parameter("cte_gain", 0.9)
        self.declare_parameter("pid_integral_limit", 0.8)
        self.declare_parameter("turn_slowdown_gain", 0.35)
        self.declare_parameter("tf_warmup_count", 5)

        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.pid_kp = float(self.get_parameter("pid_kp").value)
        self.pid_ki = float(self.get_parameter("pid_ki").value)
        self.pid_kd = float(self.get_parameter("pid_kd").value)
        self.cte_gain = float(self.get_parameter("cte_gain").value)
        self.pid_integral_limit = float(self.get_parameter("pid_integral_limit").value)
        self.turn_slowdown_gain = float(self.get_parameter("turn_slowdown_gain").value)
        self.tf_warmup_count = int(self.get_parameter("tf_warmup_count").value)

        if not self.trajectory_file:
            raise ValueError("trajectory_file parameter is required")

        self.trajectory = self._load_trajectory(self.trajectory_file)
        if len(self.trajectory) < 2:
            raise ValueError("trajectory must contain at least 2 points")

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, "/reference_trajectory", 1)
        self.target_marker_pub = self.create_publisher(Marker, "/reference_target_marker", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_target_idx = 0
        self.finished = False
        self.prev_control_error = 0.0
        self.integral_error = 0.0
        self.prev_control_time_ns: Optional[int] = None
        self._tf_warmup_remaining = self.tf_warmup_count

        self._publish_path_once()
        self.control_timer = self.create_timer(1.0 / max(self.control_rate, 1.0), self._control_step)

        self.get_logger().info(f"Loaded trajectory points: {len(self.trajectory)}")
        self.get_logger().info(f"Tracking file: {self.trajectory_file}")
        self.get_logger().info(
            "Controller: PID "
            f"(kp={self.pid_kp:.2f}, ki={self.pid_ki:.2f}, kd={self.pid_kd:.2f}, cte_gain={self.cte_gain:.2f})"
        )
        self.get_logger().info(
            f"Sync guard: waiting for {self.tf_warmup_count} consecutive valid TF reads "
            f"(tf2 timeout=0.1s) before control starts"
        )

    def _load_trajectory(self, file_path: str) -> List[Tuple[float, float, float, float]]:
        rows: List[Tuple[float, float, float, float]] = []
        with open(file_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row["x"])
                y = float(row["y"])
                yaw = float(row["yaw"])
                v_ref = float(row.get("v_ref", 0.3))
                rows.append((x, y, yaw, v_ref))
        return rows

    def _publish_path_once(self) -> None:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, _, _ in self.trajectory:
            from geometry_msgs.msg import PoseStamped

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
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

    def _find_target_index(self, rx: float, ry: float) -> int:
        if self.current_target_idx == 0:
            search_start = 0
            search_end = len(self.trajectory)
        else:
            search_start = self.current_target_idx
            search_end = min(self.current_target_idx + 400, len(self.trajectory))

        nearest_idx = search_start
        nearest_dist = float("inf")
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

        return target_idx

    def _publish_target_marker(self, x: float, y: float) -> None:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pid_trajectory_follower"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0
        self.target_marker_pub.publish(marker)

    def _stop_robot(self) -> None:
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _compute_dt(self) -> float:
        now_ns = self.get_clock().now().nanoseconds
        if self.prev_control_time_ns is None:
            self.prev_control_time_ns = now_ns
            return 1.0 / max(self.control_rate, 1.0)

        dt = max((now_ns - self.prev_control_time_ns) / 1e9, 1e-3)
        self.prev_control_time_ns = now_ns
        return dt

    def _control_step(self) -> None:
        if self.finished:
            self._stop_robot()
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            self._stop_robot()
            return

        if self._tf_warmup_remaining > 0:
            self._tf_warmup_remaining -= 1
            self.get_logger().info(
                f"TF warm-up: {self._tf_warmup_remaining} ticks remaining — holding.",
                throttle_duration_sec=0.5,
            )
            self._stop_robot()
            return

        rx, ry, ryaw = pose
        gx, gy, _, _ = self.trajectory[-1]
        if math.hypot(gx - rx, gy - ry) <= self.goal_tolerance:
            self.finished = True
            self._stop_robot()
            self.get_logger().info("Goal reached. Stopping PID trajectory follower.")
            return

        target_idx = self._find_target_index(rx, ry)
        self.current_target_idx = max(self.current_target_idx, target_idx)

        tx, ty, tyaw, v_ref = self.trajectory[self.current_target_idx]
        self._publish_target_marker(tx, ty)

        dx = tx - rx
        dy = ty - ry
        cte = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        heading_error = normalize_angle(tyaw - ryaw)
        control_error = heading_error + self.cte_gain * cte

        dt = self._compute_dt()
        self.integral_error += control_error * dt
        self.integral_error = max(-self.pid_integral_limit, min(self.pid_integral_limit, self.integral_error))
        derivative = (control_error - self.prev_control_error) / dt
        self.prev_control_error = control_error

        omega = (
            self.pid_kp * control_error
            + self.pid_ki * self.integral_error
            + self.pid_kd * derivative
        )
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))

        turn_factor = max(0.2, 1.0 - self.turn_slowdown_gain * abs(control_error))
        v_cmd = min(v_ref, self.max_linear_speed) * turn_factor
        v_cmd = max(self.min_linear_speed, v_cmd)
        if abs(control_error) > 1.2:
            v_cmd = min(v_cmd, 0.12)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PIDTrajectoryFollower()
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
