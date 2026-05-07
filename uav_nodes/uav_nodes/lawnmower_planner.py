#!/usr/bin/env python3
"""
Lawnmower Path Planner Node
Generates a zig-zag coverage path for UAV autonomous area coverage missions.

Publishes:
  /uav/coverage_path  (nav_msgs/Path)  — full lawnmower path in map frame
  /uav/waypoints      (geometry_msgs/PoseArray) — individual waypoints

Parameters:
  arena_width   (float, default 50.0)  — coverage area width  in metres
  arena_height  (float, default 50.0)  — coverage area height in metres
  overlap       (float, default 0.2)   — swath overlap fraction (0–1)
  altitude      (float, default 10.0)  — flight altitude in metres
  speed         (float, default 3.0)   — desired flight speed m/s (informational)
  swath_width   (float, default 5.0)   — camera footprint width at altitude
  frame_id      (str,   default 'map') — coordinate frame for the path
  origin_x      (float, default 0.0)   — path origin X offset
  origin_y      (float, default 0.0)   — path origin Y offset
  auto_start    (bool,  default true)  — publish path immediately on startup
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class LawnmowerPlanner(Node):
    """Generates and publishes a lawnmower coverage path."""

    def __init__(self):
        super().__init__('lawnmower_planner')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('arena_width',  50.0)
        self.declare_parameter('arena_height', 50.0)
        self.declare_parameter('overlap',       0.2)
        self.declare_parameter('altitude',     10.0)
        self.declare_parameter('speed',         3.0)
        self.declare_parameter('swath_width',   5.0)
        self.declare_parameter('frame_id',    'map')
        self.declare_parameter('origin_x',     0.0)
        self.declare_parameter('origin_y',     0.0)
        self.declare_parameter('auto_start',  True)

        # ── Publishers ────────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path,      '/uav/coverage_path', 10)
        self.wp_pub   = self.create_publisher(PoseArray, '/uav/waypoints',     10)

        # ── Generate and publish on startup ───────────────────────────────
        if self.get_parameter('auto_start').value:
            # Small delay so subscribers can connect
            self.create_timer(1.0, self._publish_once)

        self.get_logger().info('LawnmowerPlanner ready.')

    # ── Internal helpers ──────────────────────────────────────────────────

    def _generate_path(self):
        """Return list of (x, y, z, yaw) tuples for the lawnmower pattern."""
        width   = self.get_parameter('arena_width').value
        height  = self.get_parameter('arena_height').value
        overlap = self.get_parameter('overlap').value
        alt     = self.get_parameter('altitude').value
        swath   = self.get_parameter('swath_width').value
        ox      = self.get_parameter('origin_x').value
        oy      = self.get_parameter('origin_y').value

        # Effective strip spacing accounting for overlap
        step = swath * (1.0 - overlap)
        if step <= 0:
            self.get_logger().error('Invalid swath/overlap combination — step <= 0')
            return []

        waypoints = []
        x = ox
        strip = 0
        while x <= ox + width:
            if strip % 2 == 0:
                # South → North
                waypoints.append((x, oy,          alt, math.pi / 2))
                waypoints.append((x, oy + height, alt, math.pi / 2))
            else:
                # North → South
                waypoints.append((x, oy + height, alt, -math.pi / 2))
                waypoints.append((x, oy,          alt, -math.pi / 2))
            x += step
            strip += 1

        return waypoints

    def _build_path_msg(self, waypoints):
        """Convert waypoint list to nav_msgs/Path."""
        frame = self.get_parameter('frame_id').value
        now   = self.get_clock().now().to_msg()

        path = Path()
        path.header = Header(frame_id=frame, stamp=now)

        for (x, y, z, yaw) in waypoints:
            ps = PoseStamped()
            ps.header = Header(frame_id=frame, stamp=now)
            ps.pose.position    = Point(x=x, y=y, z=z)
            ps.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(ps)

        return path

    def _build_pose_array_msg(self, waypoints):
        """Convert waypoint list to geometry_msgs/PoseArray."""
        frame = self.get_parameter('frame_id').value
        now   = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.header = Header(frame_id=frame, stamp=now)

        for (x, y, z, yaw) in waypoints:
            p = Pose()
            p.position    = Point(x=x, y=y, z=z)
            p.orientation = yaw_to_quaternion(yaw)
            pa.poses.append(p)

        return pa

    def _publish_once(self):
        """Generate and publish the path once, then cancel the timer."""
        waypoints = self._generate_path()
        if not waypoints:
            return

        path_msg = self._build_path_msg(waypoints)
        wp_msg   = self._build_pose_array_msg(waypoints)

        self.path_pub.publish(path_msg)
        self.wp_pub.publish(wp_msg)

        self.get_logger().info(
            f'Published lawnmower path: {len(waypoints)} waypoints, '
            f'arena {self.get_parameter("arena_width").value}x'
            f'{self.get_parameter("arena_height").value} m, '
            f'alt {self.get_parameter("altitude").value} m'
        )

        # Cancel the one-shot timer
        self.destroy_timer(self._timer_ref)

    def create_timer(self, period, callback):
        """Override to capture timer reference for cancellation."""
        self._timer_ref = super().create_timer(period, callback)
        return self._timer_ref


def main(args=None):
    rclpy.init(args=args)
    node = LawnmowerPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
