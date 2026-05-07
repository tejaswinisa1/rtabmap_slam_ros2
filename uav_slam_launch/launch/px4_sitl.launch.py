#!/usr/bin/env python3
"""
PX4 SITL Testing Launch File — ROS2 Jazzy / Ubuntu 24.04
Launches: MAVROS (UDP) + RTAB-Map SLAM + Vision-pose bridge

Requires PX4-Autopilot running separately:
  Terminal 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
  Terminal 2: ros2 launch uav_slam_launch px4_sitl.launch.py

TF tree (SITL):
  map → odom → base_link → camera_link
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_uav_launch = get_package_share_directory('uav_slam_launch')
    params_file    = os.path.join(pkg_uav_launch, 'params', 'rtabmap_uav_params.yaml')
    cyclone_xml    = os.path.join(pkg_uav_launch, 'config', 'cyclonedds.xml')
    mavros_params  = os.path.join(pkg_uav_launch, 'config', 'mavros_params.yaml')

    return LaunchDescription([
        # ── CycloneDDS ────────────────────────────────────────────────────
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',     f'file://{cyclone_xml}'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',      '0'),

        DeclareLaunchArgument('use_sim_time',   default_value='true'),
        DeclareLaunchArgument('database_path',  default_value='~/.ros/rtabmap_sitl.db'),

        SetParameter(name='use_sim_time', value='true'),

        # ── MAVROS — SITL UDP ─────────────────────────────────────────────
        # PX4 Gazebo SITL: fcu_url = udp://:14540@localhost:14557
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                mavros_params,
                {
                    'fcu_url': 'udp://:14540@localhost:14557',
                    'gcs_url': 'udp://@localhost:14550',
                }
            ],
        ),

        # ── RGBD Odometry (Gazebo camera topics) ─────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'frame_id':        'base_link',
                    'odom_frame_id':   'odom',
                    'publish_tf':      True,
                    'approx_sync':     True,
                    'subscribe_rgbd':  False,
                    'Vis/MaxFeatures': '400',
                }
            ],
            remappings=[
                ('rgb/image',       '/camera/rgb/image_raw'),
                ('rgb/camera_info', '/camera/rgb/camera_info'),
                ('depth/image',     '/camera/depth/image_raw'),
                ('odom',            '/odom'),
            ],
        ),

        # ── RTAB-Map SLAM ─────────────────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'subscribe_depth':          True,
                    'subscribe_rgbd':           False,
                    'frame_id':                 'base_link',
                    'map_frame_id':             'map',
                    'odom_frame_id':            'odom',
                    'publish_tf':               True,
                    'database_path':            LaunchConfiguration('database_path'),
                    'Mem/IncrementalMemory':    'true',
                    'Mem/InitWMWithAllNodes':   'false',
                    'Grid/3D':                  'false',
                    'Grid/FromDepth':           'false',
                    'RGBD/CreateOccupancyGrid': 'false',
                    'RGBD/ProximityBySpace':    'false',
                    'Rtabmap/DetectionRate':    '1',
                    'Vis/MaxFeatures':          '400',
                    'Mem/STMSize':              '10',
                }
            ],
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
                'use_odom':          True,
                'frame_id':          'map',
                'covariance_check':  False,   # relaxed for SITL
                'max_publish_hz':    30.0,
            }],
        ),

        # ── Static TF: base_link → camera_link (SITL) ────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.05', '--y', '0.0', '--z', '0.02',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),
    ])
