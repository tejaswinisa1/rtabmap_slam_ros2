# UAV SLAM Stack — ROS2 Jazzy / Ubuntu 24.04

Lightweight RGB-D SLAM and autonomous coverage mission system for UAVs.
Raspberry Pi 5 + Intel RealSense D435 + PX4 + MAVROS + CycloneDDS.

> Migrated from ROS2 Humble / Ubuntu 22.04 to **ROS2 Jazzy / Ubuntu 24.04**

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Ubuntu 24.04 Installation](#3-ubuntu-2404-installation)
4. [ROS2 Jazzy Installation](#4-ros2-jazzy-installation)
5. [CycloneDDS Setup](#5-cyclonedds-setup)
6. [librealsense Installation](#6-librealsense-installation)
7. [realsense2_camera Installation](#7-realsense2_camera-installation)
8. [MAVROS Installation](#8-mavros-installation)
9. [PX4 Installation](#9-px4-installation)
10. [Workspace Build Instructions](#10-workspace-build-instructions)
11. [RTAB-Map Setup](#11-rtab-map-setup)
12. [D435 Testing](#12-d435-testing)
13. [RViz Setup](#13-rviz-setup)
14. [TF Verification](#14-tf-verification)
15. [PX4 SITL Testing](#15-px4-sitl-testing)
16. [Real Drone Deployment](#16-real-drone-deployment)
17. [Performance Optimization](#17-performance-optimization)
18. [Troubleshooting](#18-troubleshooting)
19. [Raspberry Pi 5 Optimization Guide](#19-raspberry-pi-5-optimization-guide)
20. [UAV Deployment Workflow](#20-uav-deployment-workflow)

---

## 1. System Overview

### Migration Notes (Humble → Jazzy)

| Item | ROS2 Humble (old) | ROS2 Jazzy (new) |
|------|-------------------|------------------|
| Ubuntu | 22.04 LTS | 24.04 LTS |
| Python | 3.10 | 3.12 |
| Default RMW | FastDDS | CycloneDDS |
| C++ standard | C++14 | C++17 |
| `static_transform_publisher` | positional args | `--frame-id` named args |
| `setup.py` | `packages=[pkg]` | `find_packages()` |
| `nav2_msgs` | available | removed (UAV stack) |
| `octomap_msgs` | available | removed (UAV stack) |
| `grid_map_ros` | available | removed (UAV stack) |

### Architecture Pipeline

```
Intel RealSense D435  (640×480 @ 15 fps)
        │
        ▼
realsense2_camera (ROS2 Jazzy driver)
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /camera/color/camera_info
        │
        ▼
rtabmap_odom/rgbd_odometry
  → /odom
        │
        ▼
rtabmap_slam/rtabmap  ──────────────► /map
  Lightweight mode:                   /rtabmap/odom
  400 features, 1 Hz, no 3D grid
        │
        ▼
vision_pose_bridge (uav_nodes)
  → /mavros/vision_pose/pose
        │
        ▼
MAVROS ──────────────────────────► PX4 EKF2 (vision fusion)
        │
        ▼
lawnmower_planner → /uav/coverage_path
path_optimizer    → /uav/optimized_path
orb_detector      → /uav/orb_detections
duplicate_filter  → /uav/filtered_detections
        │
        ▼
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
├── uav_slam_launch/          # Launch files, params, config
│   ├── launch/
│   │   ├── slam_only.launch.py       # D435 + SLAM (no PX4)
│   │   ├── slam_px4.launch.py        # SLAM + MAVROS + vision bridge
│   │   ├── full_uav_stack.launch.py  # Full mission stack
│   │   └── px4_sitl.launch.py        # SITL simulation
│   ├── params/
│   │   └── rtabmap_uav_params.yaml   # Pi 5-optimised RTAB-Map params
│   ├── config/
│   │   ├── cyclonedds.xml            # CycloneDDS config (Jazzy)
│   │   ├── cyclonedds_env.sh         # DDS environment setup script
│   │   ├── mavros_params.yaml        # MAVROS plugin config
│   │   ├── px4_ekf2_params.md        # PX4 EKF2 parameter guide
│   │   └── uav_rviz.rviz             # RViz2 UAV monitoring config
│   └── scripts/
│       └── benchmark_uav.sh          # Performance benchmarking
│
└── uav_nodes/                # Custom UAV Python nodes (Python 3.12)
    └── uav_nodes/
        ├── lawnmower_planner.py
        ├── path_optimizer.py
        ├── orb_detector.py
        ├── duplicate_filter.py
        └── vision_pose_bridge.py
```

---

## 2. Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Companion Computer | Raspberry Pi 5 (8 GB RAM recommended) |
| Depth Camera | Intel RealSense D435 |
| Flight Controller | Cube Orange (PX4 v1.14+) |
| Serial Link | USB-C or UART (Cube Orange ↔ Pi 5) |
| Storage | 32 GB+ microSD (A2 rated) |
| Power | 5V/5A USB-C for Pi 5 |
| Cooling | Heatsink + active fan (required) |

---

## 3. Ubuntu 24.04 Installation

Download Ubuntu 24.04 Server ARM64 from https://ubuntu.com/download/raspberry-pi

Flash with Raspberry Pi Imager, then:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  curl gnupg2 lsb-release build-essential git \
  software-properties-common

# Set locale (required for ROS2)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## 4. ROS2 Jazzy Installation

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
  python3-setuptools

sudo rosdep init
rosdep update

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. CycloneDDS Setup

ROS2 Jazzy uses CycloneDDS as the default RMW. Configure it for low-latency UAV use:

```bash
# Install CycloneDDS
sudo apt install -y \
  ros-jazzy-cyclonedds \
  ros-jazzy-rmw-cyclonedds-cpp

# Set as default RMW (add to ~/.bashrc)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Point to optimised UAV config (after workspace build)
echo 'export CYCLONEDDS_URI=file://$HOME/uav_ws/src/uav_slam_launch/config/cyclonedds.xml' >> ~/.bashrc

source ~/.bashrc
```

Verify CycloneDDS is active:

```bash
ros2 doctor --report | grep rmw
# Expected: rmw_implementation: rmw_cyclonedds_cpp
```

---

## 6. librealsense Installation

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

# Add user to plugdev group
sudo usermod -aG plugdev $USER
# Log out and back in

# Verify
rs-enumerate-devices
```

---

## 7. realsense2_camera Installation

```bash
sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description
```

---

## 8. MAVROS Installation

```bash
sudo apt install -y \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-mavros-msgs

# Install GeographicLib datasets (required)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Serial port access
sudo usermod -aG dialout $USER
```

---

## 9. PX4 Installation

Flash PX4 v1.14+ to Cube Orange via QGroundControl:
1. Download QGC: https://qgroundcontrol.com/downloads/
2. Connect Cube Orange via USB
3. Vehicle Setup → Firmware → PX4 Pro Stable v1.14+

Configure EKF2 parameters (see `config/px4_ekf2_params.md`):

```
EKF2_AID_MASK  = 24   (vision position + vision yaw)
EKF2_HGT_REF   = 3    (vision height)
EKF2_EV_DELAY  = 50   (ms)
EKF2_EV_CTRL   = 15
COM_ARM_WO_GPS = 1
```

For SITL (development machine only, not Pi 5):

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
# Build with Gazebo (Jazzy uses gz-sim, not gazebo-classic)
make px4_sitl gz_x500
```

---

## 10. Workspace Build Instructions

```bash
# Install RTAB-Map and additional deps
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

## 11. RTAB-Map Setup

RTAB-Map is installed via apt (`ros-jazzy-rtabmap-ros`). No source build needed.

Verify installation:

```bash
ros2 pkg list | grep rtabmap
# Expected:
#   rtabmap_conversions
#   rtabmap_msgs
#   rtabmap_odom
#   rtabmap_slam
#   rtabmap_sync
#   rtabmap_util
#   rtabmap_viz
```

Key optimised parameters (in `params/rtabmap_uav_params.yaml`):

```yaml
Vis/MaxFeatures:          "400"   # limit features for Pi 5
Rtabmap/DetectionRate:    "1"     # 1 Hz loop closure check
Grid/3D:                  "false" # no dense 3D grid
Grid/FromDepth:           "false" # no depth-based grid
RGBD/ProximityBySpace:    "false" # no expensive proximity search
RGBD/CreateOccupancyGrid: "false" # no occupancy grid
Mem/STMSize:              "10"    # small short-term memory
approx_sync:              true    # tolerates Pi 5 timing jitter
```

---

## 12. D435 Testing

```bash
# Plug D435 into USB 3.0 (blue port)
rs-enumerate-devices
# Expected: Intel RealSense D435

# Launch camera
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  pointcloud.enable:=false

# Verify topics
ros2 topic hz /camera/color/image_raw
# Expected: ~15.0 Hz

ros2 topic hz /camera/aligned_depth_to_color/image_raw
# Expected: ~15.0 Hz

ros2 topic list | grep camera
```

---

## 13. RViz Setup

```bash
# Launch SLAM with RViz
ros2 launch uav_slam_launch slam_only.launch.py rviz:=true

# Or open manually
rviz2 -d ~/uav_ws/src/uav_slam_launch/config/uav_rviz.rviz
```

Expected displays:
- TF tree: `map → odom → base_link → camera_link`
- Map: 2D occupancy grid
- Odometry: arrow tracking movement
- RGB Image: live D435 feed
- Coverage Path: lawnmower waypoints

---

## 14. TF Verification

```bash
# View full TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf — verify: map → odom → base_link → camera_link

# Check specific transforms
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link

# Check for TF errors
ros2 topic echo /rosout 2>/dev/null | grep -i "tf\|extrapolat"
```

Expected TF tree:
```
map → odom → base_link → camera_link → camera_color_optical_frame
```

---

## 15. PX4 SITL Testing

```bash
# Terminal 1: PX4 SITL with Gazebo (Jazzy uses gz-sim)
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: SLAM + MAVROS bridge
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh
ros2 launch uav_slam_launch px4_sitl.launch.py

# Verify MAVROS connection
ros2 topic echo /mavros/state --once
# Expected: connected: True

# Verify vision pose flowing
ros2 topic hz /mavros/vision_pose/pose
# Expected: ~15 Hz

# Arm and takeoff (OFFBOARD mode)
ros2 run mavros mavsys mode -c OFFBOARD
ros2 run mavros mavsafety arm
ros2 topic pub /mavros/setpoint_position/local \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}" \
  --rate 10
```

---

## 16. Real Drone Deployment

### Pre-flight checklist

```bash
# 1. Source environment
source ~/uav_ws/install/setup.bash
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh

# 2. Verify all topics
ros2 topic list | grep -E "camera|odom|map|mavros"

# 3. Check topic rates
ros2 topic hz /camera/color/image_raw
ros2 topic hz /odom
ros2 topic hz /mavros/vision_pose/pose

# 4. Verify TF tree
ros2 run tf2_tools view_frames

# 5. Check MAVROS
ros2 topic echo /mavros/state --once

# 6. Monitor temperature
watch -n 2 vcgencmd measure_temp
```

### Launch sequences

```bash
# SLAM only (indoor test, no PX4)
ros2 launch uav_slam_launch slam_only.launch.py

# SLAM + PX4 (hardware flight)
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Full autonomous coverage mission
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  arena_width:=50.0 arena_height:=50.0 altitude:=10.0 speed:=3.0

# Localization mode (existing map)
ros2 launch uav_slam_launch slam_only.launch.py localization:=true
```

---

## 17. Performance Optimization

### Expected CPU usage (Pi 5, 4-core, Jazzy)

| Component | CPU |
|-----------|-----|
| realsense2_camera | ~8% |
| rgbd_odometry | ~25% |
| rtabmap (1 Hz) | ~15% |
| uav_nodes (all 5) | ~5% |
| MAVROS | ~3% |
| CycloneDDS overhead | ~1% |
| **Total** | **~57%** |

### Reduce CPU if needed

```yaml
# Edit params/rtabmap_uav_params.yaml:
Vis/MaxFeatures: "200"        # reduce from 400
Rtabmap/DetectionRate: "0.5"  # reduce from 1 Hz
OdomF2M/MaxSize: "500"        # reduce from 1000
```

```bash
# Reduce camera FPS
# In slam_only.launch.py:
# rgb_camera.profile: '640x480x10'  # reduce from 15
```

### Add swap (recommended for Pi 5)

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Run benchmark

```bash
# Launch stack first, then:
bash ~/uav_ws/src/uav_slam_launch/scripts/benchmark_uav.sh 60
```

---

## 18. Troubleshooting

### CycloneDDS issues

**`RMW_IMPLEMENTATION` not set**
```bash
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh
# Or add to ~/.bashrc permanently
```

**Nodes can't discover each other**
```bash
# Check ROS_DOMAIN_ID matches on all terminals
echo $ROS_DOMAIN_ID  # should be 0
# Check cyclonedds.xml NetworkInterfaceAddress
# Change 'lo' to your actual interface (eth0, wlan0) for multi-machine
```

### TF issues

**`map → odom` not publishing**
```bash
ros2 node list | grep rtabmap
ros2 topic echo /odom --once
# Fix: ensure rgbd_odometry is running
```

**`base_link → camera_link` missing**
```bash
# Jazzy static_transform_publisher uses named args:
ros2 run tf2_ros static_transform_publisher \
  --x 0.05 --y 0 --z 0.02 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link
```

### MAVROS issues

**Not connecting to Cube Orange**
```bash
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0
# Or permanently: sudo usermod -aG dialout $USER
```

**Vision pose not accepted by PX4**
```bash
# Verify in QGC: EKF2_AID_MASK=24, EKF2_EV_CTRL=15
ros2 topic hz /mavros/vision_pose/pose  # should be >10 Hz
```

### RealSense issues

**Camera not detected**
```bash
# Use USB 3.0 (blue) port
rs-enumerate-devices
# If empty: unplug, wait 5s, replug
```

**Low FPS / frame drops**
```bash
# Check USB bandwidth
lsusb -t
# Reduce profile: rgb_camera.profile:=640x480x10
```

### Python 3.12 issues

**`setup.py` deprecation warning**
```bash
# Already fixed: setup.py uses find_packages()
# If you see warnings about distutils, install:
pip3 install --user setuptools==68.0.0
```

**Import errors in uav_nodes**
```bash
# Rebuild after any Python changes:
colcon build --packages-select uav_nodes --symlink-install
source install/setup.bash
```

### High CPU usage

```bash
# Check which process is using CPU
htop  # sort by CPU (press F6)

# Check thermal throttling
vcgencmd get_throttled
# 0x0 = no throttling
# 0x50005 = currently throttled

# Check temperature
vcgencmd measure_temp
# Should be < 80°C
```

---

## 19. Raspberry Pi 5 Optimization Guide

### Why 640×480 @ 15 fps?

The Pi 5 ARM Cortex-A76 can process ~25 ms per frame at 640×480 with 400 ORB features. At 1080p this becomes ~100 ms — too slow for stable odometry. At 30 fps the frame budget drops to 33 ms, leaving no headroom for thermal throttling.

### Why CycloneDDS over FastDDS?

CycloneDDS has lower CPU overhead for local (single-machine) communication. On Pi 5, this saves ~2-3% CPU compared to FastDDS, which matters when running near the 80% target.

### Why dense mapping is disabled?

`Grid/3D=false` and `Grid/FromDepth=false` eliminate the largest CPU consumer in RTAB-Map. For UAV coverage missions, a 2D pose graph is sufficient. Dense reconstruction can be done offline from the saved `.db` file on a more powerful machine.

### Thermal management

```bash
# Monitor continuously
watch -n 2 'vcgencmd measure_temp && vcgencmd get_throttled'

# If throttling occurs:
# 1. Add active cooling (fan)
# 2. Reduce FPS to 10
# 3. Reduce Vis/MaxFeatures to 200
# 4. Reduce Rtabmap/DetectionRate to 0.5
```

### Pi 5 boot optimizations

```bash
# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable cups
sudo systemctl disable avahi-daemon

# Set CPU governor to performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase GPU memory split (not needed for headless)
# In /boot/firmware/config.txt:
# gpu_mem=16
```

---

## 20. UAV Deployment Workflow

### Step 1 — Ground test (props off)

```bash
# Launch full stack
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Verify all systems
ros2 topic hz /camera/color/image_raw   # ~15 Hz
ros2 topic hz /odom                      # ~15 Hz
ros2 topic hz /mavros/vision_pose/pose   # ~15 Hz
ros2 topic echo /mavros/state --once     # connected: True

# Run benchmark
bash ~/uav_ws/src/uav_slam_launch/scripts/benchmark_uav.sh 30
```

### Step 2 — Static SLAM test

Move camera by hand slowly. Verify:
- `/odom` tracks movement smoothly
- `/map` builds correctly in RViz
- Loop closure detected when returning to start
- CPU stays below 80%

### Step 3 — Hover test (props on, 1 m altitude)

```bash
# Monitor during hover
ros2 topic hz /mavros/vision_pose/pose
ros2 topic echo /mavros/local_position/pose --once
watch -n 1 vcgencmd measure_temp
```

Pass criteria: stable hover, position hold ±0.3 m, CPU < 80%, temp < 75°C.

### Step 4 — Slow waypoint test

```bash
ros2 topic pub /mavros/setpoint_position/local \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 3.0}}}" \
  --rate 10
```

### Step 5 — Full autonomous coverage mission

```bash
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  fcu_url:=/dev/ttyACM0:921600 \
  arena_width:=50.0 \
  arena_height:=50.0 \
  overlap:=0.2 \
  altitude:=10.0 \
  speed:=3.0
```

Monitor:
- `/uav/coverage_path` — lawnmower path generated
- `/uav/optimized_path` — smoothed path
- `/uav/filtered_detections` — deduplicated detections
- CPU < 80%, temperature < 75°C throughout mission

---

## Quick Reference

```bash
# Environment setup (run once per terminal)
source ~/uav_ws/install/setup.bash
source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh

# SLAM only
ros2 launch uav_slam_launch slam_only.launch.py

# SLAM + RViz
ros2 launch uav_slam_launch slam_only.launch.py rviz:=true

# SLAM + PX4
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Full mission
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  arena_width:=50.0 arena_height:=50.0 altitude:=10.0

# SITL
ros2 launch uav_slam_launch px4_sitl.launch.py

# Benchmark
bash ~/uav_ws/src/uav_slam_launch/scripts/benchmark_uav.sh 60
```
