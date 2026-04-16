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


class TrajectoryFollower(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_follower")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "car_1_base_link")
        self.declare_parameter("cmd_topic", "/car_1/cmd_vel")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("lookahead_distance", 0.8)
        self.declare_parameter("max_linear_speed", 0.8)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("heading_gain", 1.0)
        # How many consecutive valid TF reads required before control starts.
        self.declare_parameter("tf_warmup_count", 5)

        self.trajectory_file = str(self.get_parameter("trajectory_file").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
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
        # Counts consecutive valid TF reads before control starts (warm-up sync guard).
        self._tf_warmup_remaining = self.tf_warmup_count
        # Tracks consecutive stale TF reads so we can warn without spamming.
        self._stale_tf_count = 0

        self._publish_path_once()
        self.control_timer = self.create_timer(1.0 / max(self.control_rate, 1.0), self._control_step)

        self.get_logger().info(f"Loaded trajectory points: {len(self.trajectory)}")
        self.get_logger().info(f"Tracking file: {self.trajectory_file}")
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
        # Path visualization is optional for control; controller uses raw trajectory list.
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
            # Use a short timeout so tf2 itself blocks and retries briefly.
            # If no valid transform arrives within the timeout, TransformException
            # is raised — no manual clock comparison needed (avoids sim/wall time mismatch).
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as ex:
            self._stale_tf_count += 1
            # Do NOT reset _tf_warmup_remaining here — just pause the countdown.
            # Resetting on every failure causes the gate to never clear during
            # the noisy SLAM startup window.
            self.get_logger().warn(f"TF lookup failed: {ex}", throttle_duration_sec=2.0)
            return None

        # Fresh transform — reset stale counter
        self._stale_tf_count = 0

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return tx, ty, yaw

    def _find_target_index(self, rx: float, ry: float) -> int:
        # On first call (warm-up just completed) or if we have no progress yet,
        # search the ENTIRE trajectory for the nearest point so the car starts
        # from the right place even if SLAM placed it slightly differently than
        # when the trajectory was recorded.
        if self.current_target_idx == 0:
            search_start = 0
            search_end = len(self.trajectory)
        else:
            # Search ahead from current position with a generous window.
            # 400 points at 0.10m spacing = 40m lookahead for the nearest search.
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

        # Walk forward from nearest until we reach lookahead distance.
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
        marker.ns = "trajectory_follower"
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
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        self.target_marker_pub.publish(marker)

    def _stop_robot(self) -> None:
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _control_step(self) -> None:
        if self.finished:
            self._stop_robot()
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            self._stop_robot()
            return

        # ── TF warm-up sync guard ─────────────────────────────────────────────
        # Wait for several consecutive valid (fresh) TF reads before issuing any
        # drive command.  This prevents the car from moving immediately on the
        # first tick when SLAM hasn't fully converged the map frame yet.
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
            self.get_logger().info("Goal reached. Stopping trajectory follower.")
            return

        target_idx = self._find_target_index(rx, ry)
        self.current_target_idx = max(self.current_target_idx, target_idx)

        tx, ty, _, v_ref = self.trajectory[self.current_target_idx]
        self._publish_target_marker(tx, ty)

        heading_to_target = math.atan2(ty - ry, tx - rx)
        alpha = normalize_angle(heading_to_target - ryaw)

        v_cmd = max(0.05, min(v_ref, self.max_linear_speed))
        if abs(alpha) > math.pi / 2.0:
            # If target is strongly behind, slow down to avoid unstable sharp turns.
            v_cmd = min(v_cmd, 0.15)

        omega = self.heading_gain * (2.0 * v_cmd * math.sin(alpha) / max(self.lookahead_distance, 1e-3))
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryFollower()
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
