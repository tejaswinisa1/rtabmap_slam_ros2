#!/usr/bin/env python3
"""
PX4 SITL Testing Launch File
Launches: Gazebo PX4 SITL + MAVROS + SLAM stack
Requires: PX4-Autopilot built with SITL support
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_uav_launch = get_package_share_directory('uav_slam_launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value='true'),
        DeclareLaunchArgument('px4_dir',
                              default_value=os.path.expanduser('~/PX4-Autopilot'),
                              description='Path to PX4-Autopilot source directory'),
        DeclareLaunchArgument('vehicle',       default_value='iris'),
        DeclareLaunchArgument('world',         default_value='empty'),

        SetParameter(name='use_sim_time', value='true'),

        # ── MAVROS (SITL UDP) ─────────────────────────────────────────────
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url':  'udp://:14540@localhost:14557',
                'gcs_url':  'udp://@localhost:14550',
                'target_system_id':    1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': [
                    'sys_status', 'sys_time', 'command',
                    'setpoint_position', 'setpoint_velocity',
                    'vision_pose_estimate', 'local_position',
                    'global_position', 'imu',
                ],
            }],
        ),

        # ── RTAB-Map SLAM (sim camera topics) ────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id':      'base_link',
                'odom_frame_id': 'odom',
                'publish_tf':    True,
                'approx_sync':   True,
                'Vis/MaxFeatures': '400',
            }],
            remappings=[
                ('rgb/image',       '/camera/rgb/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
                ('depth/image',     '/camera/depth/image_raw'),
                ('odom',            '/odom'),
            ],
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_depth':    True,
                'frame_id':           'base_link',
                'map_frame_id':       'map',
                'odom_frame_id':      'odom',
                'publish_tf':         True,
                'approx_sync':        True,
                'Rtabmap/DetectionRate': '1',
                'Vis/MaxFeatures':    '400',
                'Grid/3D':            'false',
                'Grid/FromDepth':     'false',
                'RGBD/ProximityBySpace': 'false',
                'Mem/STMSize':        '10',
            }],
            remappings=[
                ('rgb/image',       '/camera/rgb/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
                ('depth/image',     '/camera/depth/image_raw'),
                ('odom',            '/odom'),
            ],
            arguments=['-d'],
        ),

        # ── Vision-pose bridge ────────────────────────────────────────────
        Node(
            package='uav_nodes',
            executable='vision_pose_bridge',
            name='vision_pose_bridge',
            output='screen',
            parameters=[{
                'slam_pose_topic':   '/rtabmap/localization_pose',
                'mavros_pose_topic': '/mavros/vision_pose/pose',
            }],
        ),
    ])
