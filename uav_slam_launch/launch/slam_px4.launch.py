#!/usr/bin/env python3
"""
UAV SLAM + PX4/MAVROS Launch File — ROS2 Jazzy / Ubuntu 24.04
Launches: RealSense D435 + RTAB-Map + MAVROS + Vision-pose bridge

Usage:
  ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600
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
    mavros_params  = os.path.join(pkg_uav_launch, 'config', 'mavros_params.yaml')

    return LaunchDescription([
        # ── CycloneDDS ────────────────────────────────────────────────────
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',     f'file://{cyclone_xml}'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',      '0'),

        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time',   default_value='false'),
        DeclareLaunchArgument('fcu_url',        default_value='/dev/ttyACM0:921600',
                              description='FCU serial port — Cube Orange USB'),
        DeclareLaunchArgument('gcs_url',        default_value='',
                              description='GCS URL (empty = disabled)'),
        DeclareLaunchArgument('database_path',  default_value='~/.ros/rtabmap_uav.db'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ── Include SLAM-only stack ───────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_uav_launch, 'launch'), '/slam_only.launch.py'
            ]),
            launch_arguments={
                'use_sim_time':  LaunchConfiguration('use_sim_time'),
                'database_path': LaunchConfiguration('database_path'),
                'localization':  'false',
                'rviz':          'false',
            }.items(),
        ),

        # ── MAVROS ───────────────────────────────────────────────────────
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                mavros_params,
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                }
            ],
        ),

        # ── Vision-pose bridge: SLAM → MAVROS ────────────────────────────
        Node(
            package='uav_nodes',
            executable='vision_pose_bridge',
            name='vision_pose_bridge',
            output='screen',
            parameters=[{
                'slam_pose_topic':   '/rtabmap/localization_pose',
                'mavros_pose_topic': '/mavros/vision_pose/pose',
                'use_odom':          True,
                'frame_id':          'map',
                'covariance_check':  True,
                'max_covariance':    1.0,
                'max_publish_hz':    30.0,
            }],
        ),
    ])
