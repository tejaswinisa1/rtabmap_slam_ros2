#!/usr/bin/env python3
"""
UAV SLAM Only Launch File — ROS2 Jazzy / Ubuntu 24.04
Launches: RealSense D435 + RTAB-Map RGBD Odometry + RTAB-Map SLAM
Optimised for Raspberry Pi 5 — no Nav2, no costmaps, no ground-robot deps

TF tree:
  map → odom → base_link → camera_link → camera_color_optical_frame

Usage:
  # Mapping mode (default):
  ros2 launch uav_slam_launch slam_only.launch.py

  # Localization mode (existing map):
  ros2 launch uav_slam_launch slam_only.launch.py localization:=true

  # With RViz:
  ros2 launch uav_slam_launch slam_only.launch.py rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_uav_launch = get_package_share_directory('uav_slam_launch')
    params_file    = os.path.join(pkg_uav_launch, 'params', 'rtabmap_uav_params.yaml')
    cyclone_xml    = os.path.join(pkg_uav_launch, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # ── CycloneDDS (Jazzy default RMW) ───────────────────────────────
        SetEnvironmentVariable('RMW_IMPLEMENTATION',  'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',      f'file://{cyclone_xml}'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',       '0'),

        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_uav.db'),
        DeclareLaunchArgument('localization',  default_value='false',
                              description='true = localization only (no new mapping)'),
        DeclareLaunchArgument('rviz',          default_value='false',
                              description='Launch RViz2 for monitoring'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ── RealSense D435 ────────────────────────────────────────────────
        # 640x480 @ 15 fps — optimised for Pi 5
        # pointcloud.enable=false — no dense cloud, saves CPU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py'
            ]),
            launch_arguments={
                'camera_namespace':          '',
                'camera_name':               'camera',
                'align_depth.enable':        'true',
                'enable_sync':               'true',
                'rgb_camera.profile':        '640x480x15',
                'depth_module.profile':      '640x480x15',
                'enable_gyro':               'false',
                'enable_accel':              'false',
                'pointcloud.enable':         'false',
                # Disable post-processing filters — saves CPU on Pi 5
                'decimation_filter.enable':  'false',
                'spatial_filter.enable':     'false',
                'temporal_filter.enable':    'false',
                'hole_filling_filter.enable':'false',
            }.items(),
        ),

        # ── RGBD Odometry ─────────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'frame_id':              'base_link',
                    'odom_frame_id':         'odom',
                    'publish_tf':            True,
                    'approx_sync':           True,
                    'subscribe_rgbd':        False,
                    'wait_imu_to_init':      False,
                    'Vis/MaxFeatures':       '400',
                    'OdomF2M/MaxSize':       '1000',
                    'OdomF2M/MaxNewFeatures':'200',
                }
            ],
            remappings=[
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
                ('odom',            '/odom'),
            ],
        ),

        # ── RTAB-Map SLAM — mapping mode ──────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration('localization')),
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
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
                ('odom',            '/odom'),
                ('map',             '/map'),
            ],
            arguments=['-d'],
        ),

        # ── RTAB-Map SLAM — localization mode ─────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('localization')),
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
                    'Mem/IncrementalMemory':    'false',
                    'Mem/InitWMWithAllNodes':   'true',
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
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
                ('odom',            '/odom'),
                ('map',             '/map'),
            ],
        ),

        # ── Static TF: base_link → camera_link ───────────────────────────
        # Adjust x/y/z to match your physical camera mount on the drone
        # Default: camera 5 cm forward, 2 cm up from base_link origin
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

        # ── Static TF: camera_link → camera_color_optical_frame ──────────
        # RealSense D435 optical frame convention (ROS REP-103)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '-1.5707963', '--pitch', '0.0', '--yaw', '-1.5707963',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),

        # ── Optional RViz2 ────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(pkg_uav_launch, 'config', 'uav_rviz.rviz')],
        ),
    ])
