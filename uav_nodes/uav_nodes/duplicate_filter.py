#!/usr/bin/env python3
"""
Duplicate Detection Filter Node
Filters repeated detections based on spatial proximity and time.
Prevents the same object/region from being reported multiple times
during a coverage mission.

Subscribes:
  /uav/orb_detections        (geometry_msgs/PoseArray) — raw detections
  /odom                      (nav_msgs/Odometry)        — UAV position

Publishes:
  /uav/filtered_detections   (geometry_msgs/PoseArray) — deduplicated detections

Parameters:
  position_threshold  (float, default 2.0)  — spatial dedup radius in metres
  time_threshold      (float, default 5.0)  — time window for dedup in seconds
  input_topic         (str)                 — raw detections topic
  output_topic        (str)                 — filtered detections topic
  max_history         (int,   default 500)  — max stored detection positions
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class DetectionRecord:
    """Stores a detection's world position and timestamp."""
    __slots__ = ('x', 'y', 'z', 'ts')

    def __init__(self, x, y, z, ts):
        self.x  = x
        self.y  = y
        self.z  = z
        self.ts = ts


class DuplicateFilter(Node):
    """Filters duplicate detections by position and time."""

    def __init__(self):
        super().__init__('duplicate_filter')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('position_threshold', 2.0)
        self.declare_parameter('time_threshold',     5.0)
        self.declare_parameter('input_topic',        '/uav/orb_detections')
        self.declare_parameter('output_topic',       '/uav/filtered_detections')
        self.declare_parameter('max_history',        500)

        # ── State ─────────────────────────────────────────────────────────
        self._history: list[DetectionRecord] = []
        self._uav_x = 0.0
        self._uav_y = 0.0
        self._uav_z = 0.0

        # ── Subscriptions ─────────────────────────────────────────────────
        in_topic = self.get_parameter('input_topic').value
        self.sub_det  = self.create_subscription(
            PoseArray, in_topic, self._detection_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        out_topic = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(PoseArray, out_topic, 10)

        # Periodic history cleanup
        self.create_timer(10.0, self._cleanup_history)

        self.get_logger().info(
            f'DuplicateFilter ready — pos_thr={self.get_parameter("position_threshold").value} m, '
            f'time_thr={self.get_parameter("time_threshold").value} s')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._uav_x = msg.pose.pose.position.x
        self._uav_y = msg.pose.pose.position.y
        self._uav_z = msg.pose.pose.position.z

    def _detection_cb(self, msg: PoseArray):
        pos_thr  = self.get_parameter('position_threshold').value
        time_thr = self.get_parameter('time_threshold').value
        max_hist = int(self.get_parameter('max_history').value)
        now      = time.monotonic()

        filtered_poses = []

        for pose in msg.poses:
            # Convert normalised pixel coords back to approximate world coords
            # using UAV position as reference (rough estimate for dedup purposes)
            wx = self._uav_x + pose.position.x
            wy = self._uav_y + pose.position.y
            wz = self._uav_z

            if not self._is_duplicate(wx, wy, wz, now, pos_thr, time_thr):
                filtered_poses.append(pose)
                rec = DetectionRecord(wx, wy, wz, now)
                self._history.append(rec)
                # Trim history if too large
                if len(self._history) > max_hist:
                    self._history.pop(0)

        if filtered_poses:
            out = PoseArray()
            out.header = msg.header
            out.poses  = filtered_poses
            self.pub.publish(out)
            self.get_logger().debug(
                f'Passed {len(filtered_poses)}/{len(msg.poses)} detections')

    # ── Helpers ───────────────────────────────────────────────────────────

    def _is_duplicate(self, x, y, z, ts, pos_thr, time_thr) -> bool:
        for rec in self._history:
            if (ts - rec.ts) > time_thr:
                continue  # too old, skip
            dist = math.sqrt(
                (x - rec.x) ** 2 +
                (y - rec.y) ** 2 +
                (z - rec.z) ** 2
            )
            if dist < pos_thr:
                return True
        return False

    def _cleanup_history(self):
        """Remove stale records to keep memory bounded."""
        time_thr = self.get_parameter('time_threshold').value
        now      = time.monotonic()
        before   = len(self._history)
        self._history = [r for r in self._history if (now - r.ts) <= time_thr * 2]
        removed = before - len(self._history)
        if removed:
            self.get_logger().debug(f'Cleaned {removed} stale detection records')


def main(args=None):
    rclpy.init(args=args)
    node = DuplicateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
