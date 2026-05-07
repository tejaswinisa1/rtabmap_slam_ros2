#!/usr/bin/env python3
"""
ORB Feature Detection Node
Lightweight ORB-based feature extraction optimised for Raspberry Pi 5.
Detects keypoints in incoming RGB images and publishes detection results.

Subscribes:
  /camera/color/image_raw   (sensor_msgs/Image) — RGB image stream

Publishes:
  /uav/orb_detections       (geometry_msgs/PoseArray) — keypoint positions
                             Each pose: position.x/y = pixel coords (normalised 0-1),
                                        position.z   = keypoint response score

Parameters:
  max_features     (int,   default 400)                  — ORB feature limit
  scale_factor     (float, default 1.2)                  — pyramid scale factor
  n_levels         (int,   default 4)                    — pyramid levels
  image_topic      (str,   default '/camera/color/image_raw')
  detections_topic (str,   default '/uav/orb_detections')
  publish_rate_hz  (float, default 5.0)                  — max detection rate
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import time


class OrbDetector(Node):
    """Runs ORB detection on incoming images at a throttled rate."""

    def __init__(self):
        super().__init__('orb_detector')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('max_features',     400)
        self.declare_parameter('scale_factor',     1.2)
        self.declare_parameter('n_levels',         4)
        self.declare_parameter('image_topic',      '/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/uav/orb_detections')
        self.declare_parameter('publish_rate_hz',  5.0)

        # ── ORB detector ──────────────────────────────────────────────────
        self._build_orb()

        self._bridge = CvBridge()
        self._last_publish = 0.0

        img_topic  = self.get_parameter('image_topic').value
        det_topic  = self.get_parameter('detections_topic').value

        self.sub = self.create_subscription(Image, img_topic, self._image_cb, 2)
        self.pub = self.create_publisher(PoseArray, det_topic, 10)

        self.get_logger().info(
            f'OrbDetector ready — max_features={self.get_parameter("max_features").value}, '
            f'listening on {img_topic}')

    def _build_orb(self):
        """Instantiate the OpenCV ORB detector with current parameters."""
        self._orb = cv2.ORB_create(
            nfeatures=int(self.get_parameter('max_features').value),
            scaleFactor=float(self.get_parameter('scale_factor').value),
            nlevels=int(self.get_parameter('n_levels').value),
            edgeThreshold=15,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=10,
        )

    def _image_cb(self, msg: Image):
        rate = self.get_parameter('publish_rate_hz').value
        now  = time.monotonic()
        if (now - self._last_publish) < (1.0 / rate):
            return  # throttle
        self._last_publish = now

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        keypoints = self._orb.detect(gray, None)

        pa = PoseArray()
        pa.header = Header(frame_id=msg.header.frame_id, stamp=msg.header.stamp)

        for kp in keypoints:
            p = Pose()
            # Normalise pixel coordinates to [0, 1]
            p.position = Point(
                x=kp.pt[0] / w,
                y=kp.pt[1] / h,
                z=float(kp.response),
            )
            p.orientation = Quaternion(w=1.0)
            pa.poses.append(p)

        self.pub.publish(pa)

        if len(keypoints) > 0:
            self.get_logger().debug(f'Detected {len(keypoints)} ORB keypoints')


def main(args=None):
    rclpy.init(args=args)
    node = OrbDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
