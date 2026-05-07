# UAV SLAM Stack — ROS2 Jazzy / Ubuntu 24.04

Lightweight RGB-D SLAM and autonomous coverage mission system for UAVs.
**Raspberry Pi 5 + Intel RealSense D435 + PX4 + MAVROS + CycloneDDS**

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Raspberry Pi 5 Setup](#3-raspberry-pi-5-setup)
4. [Ubuntu 24.04 Setup](#4-ubuntu-2404-setup)
5. [ROS2 Jazzy Installation](#5-ros2-jazzy-installation)
6. [CycloneDDS Setup](#6-cyclonedds-setup)
7. [librealsense Installation](#7-librealsense-installation)
8. [realsense2_camera Installation](#8-realsense2_camera-installation)
9. [MAVROS Installation](#9-mavros-installation)
10. [PX4 Installation](#10-px4-installation)
11. [Workspace Build Instructions](#11-workspace-build-instructions)
12. [RTAB-Map Setup](#12-rtab-map-setup)
13. [D435 Verification](#13-d435-verification)
14. [TF Verification](#14-tf-verification)
15. [MAVROS Verification](#15-mavros-verification)
16. [PX4 SITL Testing](#16-px4-sitl-testing)
17. [RViz Testing](#17-rviz-testing)
18. [Real Drone Bench Testing](#18-real-drone-bench-testing)
19. [Hover Testing](#19-hover-testing)
20. [Autonomous Mission Testing](#20-autonomous-mission-testing)
21. [Performance Optimization Guide](#21-performance-optimization-guide)
22. [Troubleshooting Guide](#22-troubleshooting-guide)

---

## 1. System Overview

### Architecture Pipeline

```
Intel RealSense D435  (640x480 @ 15 fps)
        |
        v
realsense2_camera
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /camera/color/camera_info
        |
        v
rtabmap_odom/rgbd_odometry
  -> /odom  (nav_msgs/Odometry)
        |
        v
rtabmap_slam/rtabmap  ──────────────> /map
  Lightweight mode:                   /rtabmap/odom
  400 features, 1 Hz, no 3D grid,
  no dense cloud, no octomap
        |
        v
vision_pose_bridge  (uav_nodes)
  -> /mavros/vision_pose/pose
        |
        v
MAVROS ──────────────────────────> PX4 EKF2 (vision fusion)
        |
        v
lawnmower_planner -> /uav/coverage_path
path_optimizer    -> /uav/optimized_path
orb_detector      -> /uav/orb_detections
duplicate_filter  -> /uav/filtered_detections
        |
        v
PX4 Waypoint Navigation
```

### TF Tree

```
map
 └── odom
      └── base_link
           └── camera_link
                └── camera_color_optical_frame
```

### Package Structure

```
rtabmap_slam_ros2/
├── uav_slam_launch/
│   ├── launch/
│   │   ├── slam_only.launch.py       # D435 + SLAM (no PX4)
│   │   ├── slam_px4.launch.py        # SLAM + MAVROS + vision bridge
│   │   ├── full_uav_stack.launch.py  # Full mission stack
│   │   └── px4_sitl.launch.py        # SITL simulation
│   ├── params/
│   │   └── rtabmap_uav_params.yaml   # Pi 5-optimised RTAB-Map params
│   ├── config/
│   │   ├── cyclonedds.xml            # CycloneDDS config
│   │   ├── cyclonedds_env.sh         # DDS environment setup
│   │   ├── mavros_params.yaml        # MAVROS plugin config
│   │   ├── px4_ekf2_params.md        # PX4 EKF2 parameter guide
│   │   └── uav_rviz.rviz             # RViz2 UAV config
│   └── scripts/
│       └── benchmark_uav.sh          # Performance benchmark
│
└── uav_nodes/
    └── uav_nodes/
        ├── lawnmower_planner.py      # Coverage path generator
        ├── path_optimizer.py         # Chaikin path smoother
        ├── orb_detector.py           # Lightweight ORB detection
        ├── duplicate_filter.py       # Spatial/temporal dedup
        └── vision_pose_bridge.py     # SLAM -> MAVROS bridge
```

### Quick Launch Reference

```bash
# Source environment (run once per terminal)
source ~/uav_ws/install/setup.bash
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh

# SLAM only (indoor test, no PX4)
ros2 launch uav_slam_launch slam_only.launch.py

# SLAM + RViz
ros2 launch uav_slam_launch slam_only.launch.py rviz:=true

# SLAM + PX4 hardware
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Full autonomous coverage mission
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  arena_width:=50.0 arena_height:=50.0 altitude:=10.0 speed:=3.0

# PX4 SITL simulation
ros2 launch uav_slam_launch px4_sitl.launch.py

# Localization mode (existing map)
ros2 launch uav_slam_launch slam_only.launch.py localization:=true

# Performance benchmark
bash ~/uav_ws/src/uav_slam_launch/scripts/benchmark_uav.sh 60
```

---

## 2. Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| Companion Computer | Raspberry Pi 5 (8 GB RAM) | 4 GB minimum |
| Depth Camera | Intel RealSense D435 | USB 3.0 required |
| Flight Controller | Cube Orange | PX4 v1.14+ |
| Serial Link | USB-C or UART | Cube Orange to Pi 5 |
| Storage | 32 GB+ microSD (A2 rated) | |
| Power | 5V/5A USB-C for Pi 5 | Separate from FC |
| Cooling | Heatsink + active fan | Required for flight |

---

## 3. Raspberry Pi 5 Setup

### Flash Ubuntu 24.04

Download Ubuntu 24.04 Server ARM64 from https://ubuntu.com/download/raspberry-pi

Flash with Raspberry Pi Imager. Enable SSH during setup.

### First boot optimizations

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essentials
sudo apt install -y curl gnupg2 lsb-release build-essential git htop

# Add 2 GB swap (critical for Pi 5 with SLAM)
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable unused services to free CPU
sudo systemctl disable bluetooth cups avahi-daemon 2>/dev/null || true
```

### Verify Pi 5 hardware

```bash
# Check CPU temperature
vcgencmd measure_temp

# Check throttling status (0x0 = no throttling)
vcgencmd get_throttled

# Check available memory
free -h
```

---

## 4. Ubuntu 24.04 Setup

```bash
# Set locale (required for ROS2)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install software-properties-common
sudo apt install -y software-properties-common
```

---

## 5. ROS2 Jazzy Installation

```bash
# Add ROS2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  https://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-setuptools \
  python3-pip

sudo rosdep init
rosdep update

# Source ROS2 in every shell
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version
```

---

## 6. CycloneDDS Setup

ROS2 Jazzy uses CycloneDDS as the recommended RMW for low-latency UAV use.

```bash
# Install CycloneDDS
sudo apt install -y \
  ros-jazzy-cyclonedds \
  ros-jazzy-rmw-cyclonedds-cpp

# Set as default RMW permanently
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 doctor --report | grep rmw
# Expected: rmw_implementation: rmw_cyclonedds_cpp
```

### Source the UAV CycloneDDS config (after workspace build)

```bash
# Add to ~/.bashrc for permanent effect:
echo 'source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh' >> ~/.bashrc
source ~/.bashrc
```

The `cyclonedds_env.sh` script sets:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` pointing to the optimised `cyclonedds.xml`
- `ROS_DOMAIN_ID=0`

---

## 7. librealsense Installation

```bash
# Add Intel RealSense repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo \
  $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev

# USB access
sudo usermod -aG plugdev $USER
# Log out and back in

# Verify
rs-enumerate-devices
# Expected: Intel RealSense D435
```

---

## 8. realsense2_camera Installation

```bash
sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description
```

---

## 9. MAVROS Installation

```bash
sudo apt install -y \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-mavros-msgs

# Required GeographicLib datasets
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Serial port access
sudo usermod -aG dialout $USER
# Log out and back in
```

---

## 10. PX4 Installation

### Flash to Cube Orange

1. Download QGroundControl: https://qgroundcontrol.com/downloads/
2. Connect Cube Orange via USB
3. Vehicle Setup -> Firmware -> PX4 Pro Stable v1.14+
4. Configure EKF2 parameters (see `config/px4_ekf2_params.md`)

Key parameters:
```
EKF2_AID_MASK  = 24   (vision position + vision yaw)
EKF2_HGT_REF   = 3    (vision height)
EKF2_EV_DELAY  = 50   (ms, tune to your system)
EKF2_EV_CTRL   = 15
COM_ARM_WO_GPS = 1
EKF2_GPS_CTRL  = 0
```

### PX4 SITL (development machine only, not Pi 5)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
# Build with Gazebo (Jazzy uses gz-sim)
make px4_sitl gz_x500
```

---

## 11. Workspace Build Instructions

```bash
# Install RTAB-Map and additional dependencies
sudo apt install -y \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  python3-opencv \
  python3-numpy

# Create workspace
mkdir -p ~/uav_ws/src
cd ~/uav_ws/src

# Clone repository
git clone https://github.com/tejaswinisa1/rtabmap_slam_ros2 .

# Install ROS dependencies
cd ~/uav_ws
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "rtabmap_costmap_plugins rtabmap_demos nav2_msgs octomap_msgs grid_map_ros"

# Build UAV packages
colcon build \
  --packages-select uav_nodes uav_slam_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
  --parallel-workers 2

# Source workspace
echo "source ~/uav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Set up CycloneDDS
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh
```

---

## 12. RTAB-Map Setup

RTAB-Map is installed via apt. No source build needed.

```bash
# Verify installation
ros2 pkg list | grep rtabmap
# Expected: rtabmap_odom, rtabmap_slam, rtabmap_sync, rtabmap_util, rtabmap_msgs

# Check version
ros2 pkg xml rtabmap_slam | grep version
```

Key optimised parameters in `params/rtabmap_uav_params.yaml`:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `Vis/MaxFeatures` | 400 | Limits CPU on Pi 5 |
| `Rtabmap/DetectionRate` | 1 Hz | Biggest CPU saver |
| `Grid/3D` | false | No dense 3D grid |
| `Grid/FromDepth` | false | No depth-based grid |
| `RGBD/ProximityBySpace` | false | No expensive search |
| `RGBD/CreateOccupancyGrid` | false | No 2D costmap |
| `Mem/STMSize` | 10 | Small short-term memory |
| `approx_sync` | true | Tolerates Pi 5 jitter |
| `qos_image` | 2 (Best Effort) | Reduces backpressure |

---

## 13. D435 Verification

```bash
# Plug D435 into USB 3.0 (blue port)
rs-enumerate-devices
# Expected: Intel RealSense D435, Serial: XXXXXXXXXX

# Launch camera driver
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  pointcloud.enable:=false

# Verify topics
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
# Expected: ~15.0 Hz

ros2 topic hz /camera/aligned_depth_to_color/image_raw
# Expected: ~15.0 Hz

# Check image quality
ros2 run rqt_image_view rqt_image_view
# Select /camera/color/image_raw
```

---

## 14. TF Verification

```bash
# Launch SLAM first
ros2 launch uav_slam_launch slam_only.launch.py

# In another terminal:

# View full TF tree (generates frames.pdf)
ros2 run tf2_tools view_frames
evince frames.pdf

# Check specific transforms
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link

# Check for TF errors
ros2 topic echo /rosout 2>/dev/null | grep -i "tf\|extrapolat\|transform"
```

Expected TF tree:
```
map -> odom -> base_link -> camera_link -> camera_color_optical_frame
```

Common fixes:

**`base_link -> camera_link` missing:**
```bash
# Jazzy static_transform_publisher uses named args:
ros2 run tf2_ros static_transform_publisher \
  --x 0.05 --y 0 --z 0.02 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
```

**TF extrapolation errors:**
```bash
# Verify system clock is synced
timedatectl status
sudo apt install -y chrony && sudo systemctl enable --now chrony
```

---

## 15. MAVROS Verification

```bash
# Launch SLAM + MAVROS
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Check connection
ros2 topic echo /mavros/state --once
# Expected: connected: True, armed: False

# Check heartbeat rate
ros2 topic hz /mavros/state
# Expected: ~1 Hz

# Check vision pose flowing
ros2 topic echo /mavros/vision_pose/pose --once
ros2 topic hz /mavros/vision_pose/pose
# Expected: ~15 Hz

# Check local position (from PX4 EKF2)
ros2 topic echo /mavros/local_position/pose --once

# Check estimator status
ros2 topic echo /mavros/estimator_status --once
# pos_horiz_accuracy < 0.5 m = good fusion
```

---

## 16. PX4 SITL Testing

```bash
# Terminal 1: PX4 SITL with Gazebo
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: SLAM + MAVROS bridge
source ~/uav_ws/install/setup.bash
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh
ros2 launch uav_slam_launch px4_sitl.launch.py

# Terminal 3: Verify
ros2 topic echo /mavros/state --once
# connected: True

ros2 topic hz /mavros/vision_pose/pose
# ~15 Hz

# Arm and takeoff in OFFBOARD mode
ros2 run mavros mavsys mode -c OFFBOARD
ros2 run mavros mavsafety arm
ros2 topic pub /mavros/setpoint_position/local \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}" \
  --rate 10
```

---

## 17. RViz Testing

```bash
# Launch SLAM with RViz
ros2 launch uav_slam_launch slam_only.launch.py rviz:=true

# Or open manually
rviz2 -d ~/uav_ws/src/uav_slam_launch/config/uav_rviz.rviz
```

Expected displays:
- **TF**: `map -> odom -> base_link -> camera_link`
- **Map**: 2D occupancy grid building as you move
- **Odometry**: arrow tracking camera movement
- **RGB Image**: live D435 color feed
- **Coverage Path**: lawnmower waypoints (after full_uav_stack launch)

---

## 18. Real Drone Bench Testing

**Always test with props OFF first.**

```bash
# 1. Source environment
source ~/uav_ws/install/setup.bash
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh

# 2. Launch full stack
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# 3. Verify all topics (in another terminal)
ros2 topic hz /camera/color/image_raw   # ~15 Hz
ros2 topic hz /odom                      # ~15 Hz
ros2 topic hz /mavros/vision_pose/pose   # ~15 Hz
ros2 topic echo /mavros/state --once     # connected: True

# 4. Verify TF tree
ros2 run tf2_tools view_frames

# 5. Run 60-second benchmark
bash ~/uav_ws/src/uav_slam_launch/scripts/benchmark_uav.sh 60

# 6. Monitor temperature
watch -n 2 'vcgencmd measure_temp && vcgencmd get_throttled'
```

Pass criteria:
- All topics publishing at expected rates
- TF tree complete: `map -> odom -> base_link -> camera_link`
- MAVROS connected
- CPU < 80%
- Temperature < 75 degrees C

---

## 19. Hover Testing

**Props on. Low altitude. Safety pilot present.**

```bash
# Launch full stack
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Monitor during hover
ros2 topic hz /mavros/vision_pose/pose   # must be >10 Hz
ros2 topic echo /mavros/local_position/pose --once
watch -n 1 vcgencmd measure_temp
```

Pass criteria:
- Stable hover with vision fusion active
- Position hold within +/- 0.3 m
- CPU < 80%, temperature < 75 degrees C
- No TF extrapolation errors

---

## 20. Autonomous Mission Testing

```bash
# Launch full mission stack
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  fcu_url:=/dev/ttyACM0:921600 \
  arena_width:=20.0 \
  arena_height:=20.0 \
  overlap:=0.2 \
  altitude:=5.0 \
  speed:=2.0

# Monitor mission topics
ros2 topic echo /uav/coverage_path --once | head -20
ros2 topic echo /uav/optimized_path --once | head -20
ros2 topic echo /uav/filtered_detections

# Run benchmark during mission
bash ~/uav_ws/src/uav_slam_launch/scripts/benchmark_uav.sh 120
```

---

## 21. Performance Optimization Guide

### Expected CPU usage (Pi 5, 4-core, Jazzy + CycloneDDS)

| Component | CPU |
|-----------|-----|
| realsense2_camera | ~8% |
| rgbd_odometry | ~25% |
| rtabmap (1 Hz) | ~15% |
| uav_nodes (all 5) | ~5% |
| MAVROS | ~3% |
| CycloneDDS | ~1% |
| **Total** | **~57%** |

### Reduce CPU if needed

Edit `params/rtabmap_uav_params.yaml`:

```yaml
# Step 1: Reduce features
Vis/MaxFeatures: "200"

# Step 2: Reduce detection rate
Rtabmap/DetectionRate: "0.5"

# Step 3: Reduce odometry map size
OdomF2M/MaxSize: "500"
```

Reduce camera FPS in `slam_only.launch.py`:
```python
'rgb_camera.profile': '640x480x10',   # reduce from 15
'depth_module.profile': '640x480x10',
```

### Thermal management

```bash
# Monitor continuously
watch -n 2 'vcgencmd measure_temp && vcgencmd get_throttled'

# If throttling (get_throttled != 0x0):
# 1. Add active cooling (fan)
# 2. Reduce FPS to 10
# 3. Reduce Vis/MaxFeatures to 200
# 4. Reduce Rtabmap/DetectionRate to 0.5
```

### Memory optimization

```bash
# Check current usage
free -h

# Add swap if not done
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## 22. Troubleshooting Guide

### CycloneDDS not active

```bash
echo $RMW_IMPLEMENTATION
# If empty: source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh
ros2 doctor --report | grep rmw
```

### Camera not detected

```bash
# Use USB 3.0 (blue) port
rs-enumerate-devices
# If empty: unplug, wait 5s, replug
lsusb | grep Intel
# Check USB bandwidth: lsusb -t
```

### MAVROS not connecting

```bash
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0
# Permanent fix:
sudo usermod -aG dialout $USER
```

### Vision pose not accepted by PX4

```bash
# Verify in QGC: EKF2_AID_MASK=24, EKF2_EV_CTRL=15
ros2 topic hz /mavros/vision_pose/pose  # must be >10 Hz
# Increase EKF2_EV_DELAY by 10ms increments until stable
```

### TF extrapolation errors

```bash
# Check clock sync
timedatectl status
sudo systemctl enable --now chrony
# Verify approx_sync=true in params file
```

### High CPU usage

```bash
htop  # sort by CPU (F6)
vcgencmd get_throttled  # check thermal throttling
# Apply reductions from Performance Optimization section
```

### RTAB-Map crashes (OOM)

```bash
# Verify swap is active
swapon --show
# Reduce memory params:
# Mem/STMSize: "5"
# Vis/MaxFeatures: "200"
# OdomF2M/MaxSize: "500"
```

### Python 3.12 import errors

```bash
# Rebuild after any Python changes
colcon build --packages-select uav_nodes --symlink-install
source ~/uav_ws/install/setup.bash
```
