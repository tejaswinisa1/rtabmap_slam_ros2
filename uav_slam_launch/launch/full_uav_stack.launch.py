#!/usr/bin/env python3
"""
Full UAV Stack Launch File
Launches: SLAM + PX4/MAVROS + Lawnmower Planner + ORB Detection + Duplicate Filter
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_uav_launch = get_package_share_directory('uav_slam_launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',   default_value='false'),
        DeclareLaunchArgument('fcu_url',        default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('gcs_url',        default_value=''),
        DeclareLaunchArgument('database_path',  default_value='~/.ros/rtabmap_uav.db'),
        # Coverage mission parameters
        DeclareLaunchArgument('arena_width',    default_value='50.0',  description='Coverage area width  (m)'),
        DeclareLaunchArgument('arena_height',   default_value='50.0',  description='Coverage area height (m)'),
        DeclareLaunchArgument('overlap',        default_value='0.2',   description='Swath overlap 0-1'),
        DeclareLaunchArgument('altitude',       default_value='10.0',  description='Mission altitude (m)'),
        DeclareLaunchArgument('speed',          default_value='3.0',   description='Mission speed (m/s)'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ── SLAM + MAVROS ─────────────────────────────────────────────────
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
                'frame_id':     'map',
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
            }],
        ),

        # ── ORB Detection ─────────────────────────────────────────────────
        Node(
            package='uav_nodes',
            executable='orb_detector',
            name='orb_detector',
            output='screen',
            parameters=[{
                'max_features':    400,
                'scale_factor':    1.2,
                'n_levels':        4,
                'image_topic':     '/camera/color/image_raw',
                'detections_topic': '/uav/orb_detections',
            }],
        ),

        # ── Duplicate Filter ──────────────────────────────────────────────
        Node(
            package='uav_nodes',
            executable='duplicate_filter',
            name='duplicate_filter',
            output='screen',
            parameters=[{
                'position_threshold': 2.0,   # metres
                'time_threshold':     5.0,   # seconds
                'input_topic':        '/uav/orb_detections',
                'output_topic':       '/uav/filtered_detections',
            }],
        ),
    ])
