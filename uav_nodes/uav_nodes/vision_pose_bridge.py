#!/usr/bin/env python3
"""
Vision Pose Bridge Node — ROS2 Jazzy / Python 3.12
Bridges RTAB-Map SLAM pose to MAVROS for PX4 EKF2 external vision fusion.

Subscribes to BOTH RTAB-Map pose outputs:
  /rtabmap/odom               (nav_msgs/Odometry)                  [default]
  /rtabmap/localization_pose  (geometry_msgs/PoseWithCovarianceStamped)

Publishes:
  /mavros/vision_pose/pose    (geometry_msgs/PoseStamped)

Parameters:
  slam_pose_topic   (str,   default '/rtabmap/localization_pose')
  mavros_pose_topic (str,   default '/mavros/vision_pose/pose')
  use_odom          (bool,  default true)  — subscribe to /rtabmap/odom
  frame_id          (str,   default 'map')
  covariance_check  (bool,  default true)
  max_covariance    (float, default 1.0)
  max_publish_hz    (float, default 30.0)  — rate-limit output to MAVROS
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class VisionPoseBridge(Node):

    def __init__(self):
        super().__init__('vision_pose_bridge')

        self.declare_parameter('slam_pose_topic',   '/rtabmap/localization_pose')
        self.declare_parameter('mavros_pose_topic', '/mavros/vision_pose/pose')
        self.declare_parameter('use_odom',          True)
        self.declare_parameter('frame_id',          'map')
        self.declare_parameter('covariance_check',  True)
        self.declare_parameter('max_covariance',    1.0)
        self.declare_parameter('max_publish_hz',    30.0)

        slam_topic   = self.get_parameter('slam_pose_topic').value
        mavros_topic = self.get_parameter('mavros_pose_topic').value
        use_odom     = self.get_parameter('use_odom').value

        self._last_pub  = 0.0
        self._count     = 0

        self.pub = self.create_publisher(PoseStamped, mavros_topic, 10)

        # Subscribe to /rtabmap/odom (always published by RTAB-Map)
        if use_odom:
            self.sub_odom = self.create_subscription(
                Odometry, '/rtabmap/odom', self._odom_cb, 10)
            self.get_logger().info('Subscribed to /rtabmap/odom')

        # Subscribe to /rtabmap/localization_pose (localization mode)
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, slam_topic, self._pose_cov_cb, 10)
        self.get_logger().info(
            f'VisionPoseBridge: {slam_topic} + /rtabmap/odom → {mavros_topic}')

    def _rate_ok(self) -> bool:
        hz  = self.get_parameter('max_publish_hz').value
        now = time.monotonic()
        if (now - self._last_pub) < (1.0 / max(hz, 0.1)):
            return False
        self._last_pub = now
        return True

    def _pose_cov_cb(self, msg: PoseWithCovarianceStamped):
        if not self._rate_ok():
            return
        if self.get_parameter('covariance_check').value:
            cov     = msg.pose.covariance
            max_cov = self.get_parameter('max_covariance').value
            if cov[0] > max_cov or cov[7] > max_cov or cov[14] > max_cov:
                self.get_logger().warn(
                    f'High covariance pose skipped: '
                    f'x={cov[0]:.3f} y={cov[7]:.3f} z={cov[14]:.3f}',
                    throttle_duration_sec=5.0)
                return
        out = PoseStamped()
        out.header          = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.pose            = msg.pose.pose
        self._publish(out)

    def _odom_cb(self, msg: Odometry):
        if not self._rate_ok():
            return
        out = PoseStamped()
        out.header          = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.pose            = msg.pose.pose
        self._publish(out)

    def _publish(self, msg: PoseStamped):
        self.pub.publish(msg)
        self._count += 1
        if self._count % 300 == 0:
            self.get_logger().info(
                f'[vision_pose_bridge] {self._count} poses forwarded to MAVROS')


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
