#!/usr/bin/env python3
"""
Path Optimizer Node
Smooths a nav_msgs/Path by:
  1. Removing redundant waypoints closer than min_waypoint_distance
  2. Applying Chaikin corner-cutting to soften sharp turns

Subscribes:
  /uav/coverage_path        (nav_msgs/Path) — raw lawnmower path

Publishes:
  /uav/optimized_path       (nav_msgs/Path) — smoothed path

Parameters:
  min_waypoint_distance  (float, default 1.0)  — prune waypoints closer than this (m)
  turn_smoothing_factor  (float, default 0.5)  — Chaikin ratio (0 = no smoothing, 1 = max)
  smoothing_iterations   (int,   default 2)    — number of Chaikin passes
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def _dist(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    dz = a.pose.position.z - b.pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _lerp_pose(a: PoseStamped, b: PoseStamped, t: float) -> PoseStamped:
    """Linear interpolation between two PoseStamped positions."""
    ps = PoseStamped()
    ps.header = a.header
    ps.pose.position.x = a.pose.position.x + t * (b.pose.position.x - a.pose.position.x)
    ps.pose.position.y = a.pose.position.y + t * (b.pose.position.y - a.pose.position.y)
    ps.pose.position.z = a.pose.position.z + t * (b.pose.position.z - a.pose.position.z)
    # Keep orientation from the first pose (heading will be recalculated downstream)
    ps.pose.orientation = a.pose.orientation
    return ps


def _prune_waypoints(poses, min_dist: float):
    """Remove waypoints that are too close together."""
    if not poses:
        return poses
    pruned = [poses[0]]
    for p in poses[1:]:
        if _dist(pruned[-1], p) >= min_dist:
            pruned.append(p)
    # Always keep the last waypoint
    if pruned[-1] is not poses[-1]:
        pruned.append(poses[-1])
    return pruned


def _chaikin_smooth(poses, ratio: float, iterations: int):
    """Apply Chaikin corner-cutting algorithm to soften turns."""
    for _ in range(iterations):
        if len(poses) < 3:
            break
        smoothed = [poses[0]]  # keep start
        for i in range(len(poses) - 1):
            q = _lerp_pose(poses[i], poses[i + 1], ratio)
            r = _lerp_pose(poses[i], poses[i + 1], 1.0 - ratio)
            smoothed.append(q)
            smoothed.append(r)
        smoothed.append(poses[-1])  # keep end
        poses = smoothed
    return poses


class PathOptimizer(Node):
    """Subscribes to a raw path and publishes a smoothed version."""

    def __init__(self):
        super().__init__('path_optimizer')

        self.declare_parameter('min_waypoint_distance', 1.0)
        self.declare_parameter('turn_smoothing_factor', 0.5)
        self.declare_parameter('smoothing_iterations',  2)

        self.sub = self.create_subscription(
            Path, '/uav/coverage_path', self._path_callback, 10)
        self.pub = self.create_publisher(
            Path, '/uav/optimized_path', 10)

        self.get_logger().info('PathOptimizer ready.')

    def _path_callback(self, msg: Path):
        min_dist   = self.get_parameter('min_waypoint_distance').value
        ratio      = self.get_parameter('turn_smoothing_factor').value
        iterations = int(self.get_parameter('smoothing_iterations').value)

        poses = msg.poses

        # Step 1: prune redundant waypoints
        poses = _prune_waypoints(poses, min_dist)

        # Step 2: smooth turns
        if ratio > 0.0:
            poses = _chaikin_smooth(poses, ratio, iterations)

        out = Path()
        out.header = msg.header
        out.poses  = poses

        self.pub.publish(out)
        self.get_logger().info(
            f'Optimized path: {len(msg.poses)} → {len(poses)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = PathOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
