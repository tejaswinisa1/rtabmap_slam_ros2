from setuptools import setup, find_packages

package_name = 'uav_nodes'

setup(
    name=package_name,
    version='1.0.0',
    # find_packages() required for Python 3.12 (Ubuntu 24.04 / ROS2 Jazzy)
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Team',
    maintainer_email='uav@example.com',
    description='UAV-specific ROS2 nodes for SLAM-based autonomous coverage missions',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lawnmower_planner  = uav_nodes.lawnmower_planner:main',
            'path_optimizer     = uav_nodes.path_optimizer:main',
            'orb_detector       = uav_nodes.orb_detector:main',
            'duplicate_filter   = uav_nodes.duplicate_filter:main',
            'vision_pose_bridge = uav_nodes.vision_pose_bridge:main',
        ],
    },
)
