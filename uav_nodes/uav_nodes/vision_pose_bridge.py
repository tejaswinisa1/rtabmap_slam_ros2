#!/usr/bin/env python3
"""
Vision Pose Bridge Node
Bridges RTAB-Map SLAM pose output to MAVROS vision_pose/pose topic
for PX4 EKF2 external vision fusion.

Subscribes:
  /rtabmap/localization_pose  (geometry_msgs/PoseWithCovarianceStamped)
  — or —
  /rtabmap/odom               (nav_msgs/Odometry)

Publishes:
  /mavros/vision_pose/pose    (geometry_msgs/PoseStamped)

Parameters:
  slam_pose_topic    (str, default '/rtabmap/localization_pose')
  mavros_pose_topic  (str, default '/mavros/vision_pose/pose')
  use_odom           (bool, default false) — subscribe to /rtabmap/odom instead
  frame_id           (str, default 'map')
  child_frame_id     (str, default 'base_link')
  covariance_check   (bool, default true) — skip poses with high covariance
  max_covariance     (float, default 1.0) — max acceptable position covariance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class VisionPoseBridge(Node):
    """Forwards SLAM pose to MAVROS for PX4 EKF2 vision fusion."""

    def __init__(self):
        super().__init__('vision_pose_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('slam_pose_topic',   '/rtabmap/localization_pose')
        self.declare_parameter('mavros_pose_topic', '/mavros/vision_pose/pose')
        self.declare_parameter('use_odom',          False)
        self.declare_parameter('frame_id',          'map')
        self.declare_parameter('child_frame_id',    'base_link')
        self.declare_parameter('covariance_check',  True)
        self.declare_parameter('max_covariance',    1.0)

        slam_topic   = self.get_parameter('slam_pose_topic').value
        mavros_topic = self.get_parameter('mavros_pose_topic').value
        use_odom     = self.get_parameter('use_odom').value

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(PoseStamped, mavros_topic, 10)

        # ── Subscriber ────────────────────────────────────────────────────
        if use_odom:
            self.sub = self.create_subscription(
                Odometry, '/rtabmap/odom', self._odom_cb, 10)
            self.get_logger().info(f'Bridging /rtabmap/odom → {mavros_topic}')
        else:
            self.sub = self.create_subscription(
                PoseWithCovarianceStamped, slam_topic, self._pose_cov_cb, 10)
            self.get_logger().info(f'Bridging {slam_topic} → {mavros_topic}')

        self._msg_count = 0

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _pose_cov_cb(self, msg: PoseWithCovarianceStamped):
        """Handle PoseWithCovarianceStamped from RTAB-Map."""
        if self.get_parameter('covariance_check').value:
            max_cov = self.get_parameter('max_covariance').value
            # Check diagonal elements of 6x6 covariance (position: indices 0,7,14)
            cov = msg.pose.covariance
            if cov[0] > max_cov or cov[7] > max_cov or cov[14] > max_cov:
                self.get_logger().warn(
                    f'Skipping high-covariance pose: '
                    f'[{cov[0]:.3f}, {cov[7]:.3f}, {cov[14]:.3f}]')
                return

        out = PoseStamped()
        out.header       = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.pose         = msg.pose.pose
        self._publish(out)

    def _odom_cb(self, msg: Odometry):
        """Handle Odometry from RTAB-Map."""
        out = PoseStamped()
        out.header          = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.pose            = msg.pose.pose
        self._publish(out)

    def _publish(self, msg: PoseStamped):
        self.pub.publish(msg)
        self._msg_count += 1
        if self._msg_count % 100 == 0:
            self.get_logger().info(
                f'Vision pose bridge: {self._msg_count} poses forwarded to MAVROS')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
