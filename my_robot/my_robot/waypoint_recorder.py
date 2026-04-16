#!/usr/bin/env python3

import csv
import os
from datetime import datetime
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class WaypointRecorder(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_recorder")

        default_name = f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        default_path = f"/sim_ws/src/my_robot/maps/{default_name}"

        self.declare_parameter("output_file", default_path)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_topic", "/waypoint_markers")

        self.output_file = str(self.get_parameter("output_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        marker_topic = str(self.get_parameter("marker_topic").value)

        self._waypoints: List[Tuple[float, float, float]] = []

        self._clicked_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self._on_clicked_point,
            10,
        )
        self._markers_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.get_logger().info("Waypoint recorder started")
        self.get_logger().info("Click points in RViz using 'Publish Point'")
        self.get_logger().info("Press Ctrl+C to save and exit")
        self.get_logger().info(f"Output CSV: {self.output_file}")

    def _on_clicked_point(self, msg: PointStamped) -> None:
        # Keep all points in map frame so the trajectory generator uses one frame.
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"Ignoring point in frame '{msg.header.frame_id}', expected '{self.frame_id}'"
            )
            return

        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)
        self._waypoints.append((x, y, z))

        idx = len(self._waypoints)
        self.get_logger().info(f"Waypoint {idx}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        self._publish_markers()

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()

        points_marker = Marker()
        points_marker.header.frame_id = self.frame_id
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "waypoints"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.18
        points_marker.scale.y = 0.18
        points_marker.scale.z = 0.18
        points_marker.color.r = 0.1
        points_marker.color.g = 0.9
        points_marker.color.b = 0.2
        points_marker.color.a = 1.0
        points_marker.pose.orientation.w = 1.0

        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "waypoints"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.06
        line_marker.color.r = 0.1
        line_marker.color.g = 0.7
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0

        for x, y, z in self._waypoints:
            pt = PointStamped().point
            pt.x = x
            pt.y = y
            pt.z = z
            points_marker.points.append(pt)
            line_marker.points.append(pt)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)
        self._markers_pub.publish(marker_array)

    def save_csv(self) -> str:
        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        with open(self.output_file, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["id", "x", "y", "z", "yaw", "v_hint"])
            for i, (x, y, z) in enumerate(self._waypoints):
                writer.writerow([i, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", "", ""])

        return self.output_file


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        output_file = node.save_csv()
        node.get_logger().info(
            f"Saved {len(node._waypoints)} waypoint(s) to: {output_file}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
