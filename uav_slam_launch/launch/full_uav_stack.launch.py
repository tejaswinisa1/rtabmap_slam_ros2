#!/usr/bin/env python3
"""
Full UAV Stack Launch File — ROS2 Jazzy / Ubuntu 24.04
Launches: SLAM + PX4/MAVROS + Lawnmower Planner + Path Optimizer
          + ORB Detection + Duplicate Filter

Usage:
  ros2 launch uav_slam_launch full_uav_stack.launch.py \
    arena_width:=50.0 arena_height:=50.0 altitude:=10.0 speed:=3.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_uav_launch = get_package_share_directory('uav_slam_launch')
    cyclone_xml    = os.path.join(pkg_uav_launch, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # ── CycloneDDS ────────────────────────────────────────────────────
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',     f'file://{cyclone_xml}'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',      '0'),

        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('fcu_url',       default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('gcs_url',       default_value=''),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_uav.db'),
        DeclareLaunchArgument('arena_width',   default_value='50.0',
                              description='Coverage area width (m)'),
        DeclareLaunchArgument('arena_height',  default_value='50.0',
                              description='Coverage area height (m)'),
        DeclareLaunchArgument('overlap',       default_value='0.2',
                              description='Swath overlap fraction 0-1'),
        DeclareLaunchArgument('altitude',      default_value='10.0',
                              description='Mission altitude (m)'),
        DeclareLaunchArgument('speed',         default_value='3.0',
                              description='Mission speed (m/s)'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ── SLAM + MAVROS stack ───────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_uav_launch, 'launch'), '/slam_px4.launch.py'
            ]),
            launch_arguments={
                'use_sim_time':  LaunchConfiguration('use_sim_time'),
                'fcu_url':       LaunchConfiguration('fcu_url'),
                'gcs_url':       LaunchConfiguration('gcs_url'),
                'database_path': LaunchConfiguration('database_path'),
            }.items(),
        ),

        # ── Lawnmower Path Planner ────────────────────────────────────────
        Node(
            package='uav_nodes',
            executable='lawnmower_planner',
            name='lawnmower_planner',
            output='screen',
            parameters=[{
                'arena_width':  LaunchConfiguration('arena_width'),
                'arena_height': LaunchConfiguration('arena_height'),
                'overlap':      LaunchConfiguration('overlap'),
                'altitude':     LaunchConfiguration('altitude'),
                'speed':        LaunchConfiguration('speed'),
                'swath_width':  5.0,
                'frame_id':     'map',
                'auto_start':   True,
            }],
        ),

        # ── Path Optimizer ────────────────────────────────────────────────
        Node(
            package='uav_nodes',
            executable='path_optimizer',
            name='path_optimizer',
            output='screen',
            parameters=[{
                'min_waypoint_distance': 1.0,
                'turn_smoothing_factor': 0.5,
                'smoothing_iterations':  2,
            }],
        ),

        # ── ORB Detection (throttled to 5 Hz for Pi 5) ───────────────────
        Node(
            package='uav_nodes',
            executable='orb_detector',
            name='orb_detector',
            output='screen',
            parameters=[{
                'max_features':     400,
                'scale_factor':     1.2,
                'n_levels':         4,
                'image_topic':      '/camera/color/image_raw',
                'detections_topic': '/uav/orb_detections',
                'publish_rate_hz':  5.0,
            }],
        ),

        # ── Duplicate Filter ──────────────────────────────────────────────
        Node(
            package='uav_nodes',
            executable='duplicate_filter',
            name='duplicate_filter',
            output='screen',
            parameters=[{
                'position_threshold': 2.0,
                'time_threshold':     5.0,
                'input_topic':        '/uav/orb_detections',
                'output_topic':       '/uav/filtered_detections',
                'max_history':        500,
            }],
        ),
    ])
