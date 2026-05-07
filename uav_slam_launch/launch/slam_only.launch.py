#!/usr/bin/env python3
"""
UAV SLAM Only Launch File
Launches: RealSense D435 + RTAB-Map RGBD Odometry + RTAB-Map SLAM
Optimized for Raspberry Pi 5
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
    params_file = os.path.join(pkg_uav_launch, 'params', 'rtabmap_uav_params.yaml')

    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_uav.db'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = localization only, false = mapping'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # ── RealSense D435 ────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py'
            ]),
            launch_arguments={
                'camera_namespace': '',
                'camera_name': 'camera',
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                # 640x480 @ 15 fps — lightweight for Pi 5
                'rgb_camera.profile': '640x480x15',
                'depth_module.profile': '640x480x15',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'pointcloud.enable': 'false',
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
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'publish_tf': True,
                    'approx_sync': True,
                    'subscribe_rgbd': False,
                }
            ],
            remappings=[
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
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
                    'subscribe_depth': True,
                    'subscribe_rgbd': False,
                    'frame_id': 'base_link',
                    'map_frame_id': 'map',
                    'odom_frame_id': 'odom',
                    'publish_tf': True,
                    'database_path': LaunchConfiguration('database_path'),
                    'Mem/IncrementalMemory': 'true',
                    'Mem/InitWMWithAllNodes': 'false',
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

        # ── Static TF: base_link → camera_link ───────────────────────────
        # Adjust xyz/rpy to match your physical camera mount
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0.05', '0.0', '0.02',   # x y z  (metres)
                '0.0',  '0.0', '0.0',    # roll pitch yaw
                'base_link', 'camera_link'
            ],
        ),
    ])
