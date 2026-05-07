#!/usr/bin/env python3
"""
UAV SLAM + PX4/MAVROS Launch File
Launches: RealSense D435 + RTAB-Map + MAVROS + Vision-pose bridge
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fcu_url',      default_value='/dev/ttyACM0:921600',
                              description='FCU serial port for MAVROS'),
        DeclareLaunchArgument('gcs_url',      default_value='',
                              description='GCS URL for MAVROS (empty = disabled)'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_uav.db'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ── Include SLAM-only stack ───────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_uav_launch, 'launch'), '/slam_only.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'database_path': LaunchConfiguration('database_path'),
            }.items(),
        ),

        # ── MAVROS ───────────────────────────────────────────────────────
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url':  LaunchConfiguration('fcu_url'),
                'gcs_url':  LaunchConfiguration('gcs_url'),
                'target_system_id':    1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                # Enable vision pose plugin
                'plugin_allowlist': [
                    'sys_status', 'sys_time', 'command',
                    'setpoint_position', 'setpoint_velocity',
                    'vision_pose_estimate', 'local_position',
                    'global_position', 'imu',
                ],
            }],
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
                'frame_id':          'map',
                'child_frame_id':    'base_link',
            }],
        ),
    ])
