# UAV SLAM Stack — Raspberry Pi 5 + Intel RealSense D435 + PX4

> Lightweight RGB-D SLAM and autonomous coverage mission system for UAVs.
> Built on ROS 2 Humble, RTAB-Map, MAVROS, and PX4.

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Architecture Overview](#2-architecture-overview)
3. [Installation Guide](#3-installation-guide)
4. [Dependency Installation](#4-dependency-installation)
5. [RealSense D435 Setup](#5-realsense-d435-setup)
6. [RTAB-Map SLAM Testing](#6-rtab-map-slam-testing)
7. [PX4 SITL Testing](#7-px4-sitl-testing)
8. [Drone Hardware Testing](#8-drone-hardware-testing)
9. [Performance Optimization](#9-performance-optimization)
10. [Troubleshooting](#10-troubleshooting)
11. [UAV-Specific Design Notes](#11-uav-specific-design-notes)
12. [Package Structure](#12-package-structure)

---

## 1. System Requirements

### Hardware

| Component | Specification |
|-----------|--------------|
| Companion Computer | Raspberry Pi 5 (8 GB RAM recommended) |
| Depth Camera | Intel RealSense D435 |
| Flight Controller | Cube Orange (or any PX4-compatible FC) |
| Autopilot Firmware | PX4 v1.14+ |
| Serial Link | USB or UART (Cube Orange ↔ Pi 5) |
| Storage | 32 GB+ microSD (Class 10 / A2) |

### Software

| Software | Version |
|----------|---------|
| Operating System | Ubuntu 22.04 LTS (64-bit ARM) |
| ROS 2 | Humble Hawksbill |
| RTAB-Map | 0.21+ |
| Intel RealSense SDK | librealsense2 2.54+ |
| MAVROS | 2.x (ROS 2) |
| PX4 Autopilot | v1.14+ |
| Python | 3.10+ |
| OpenCV | 4.5+ |

---

## 2. Architecture Overview

```
Intel RealSense D435
        │  640×480 @ 15 fps  (RGB + Aligned Depth)
        ▼
realsense2_camera (ROS 2 driver)
        │  /camera/color/image_raw
        │  /camera/aligned_depth_to_color/image_raw
        │  /camera/color/camera_info
        ▼
rtabmap_odom/rgbd_odometry
        │  /odom  (nav_msgs/Odometry)
        ▼
rtabmap_slam/rtabmap  ──── Lightweight SLAM ────►  /map  (nav_msgs/OccupancyGrid)
        │                  (optimised for Pi 5)     /rtabmap/localization_pose
        ▼
vision_pose_bridge  (uav_nodes)
        │  geometry_msgs/PoseStamped
        ▼
MAVROS  ──────────────────────────────────────►  /mavros/vision_pose/pose
        │
        ▼
PX4 EKF2  (vision fusion)
        │
        ▼
lawnmower_planner  (uav_nodes)
        │  /uav/coverage_path
        ▼
path_optimizer  (uav_nodes)
        │  /uav/optimized_path
        ▼
orb_detector  (uav_nodes)
        │  /uav/orb_detections
        ▼
duplicate_filter  (uav_nodes)
        │  /uav/filtered_detections
        ▼
PX4 Waypoint Navigation
```

### TF Tree

```
map
 └── odom
      └── base_link
           └── camera_link
```

---

## 3. Installation Guide

### 3.1 Ubuntu 22.04 on Raspberry Pi 5

Download the official Ubuntu 22.04 Server ARM64 image from
[ubuntu.com/download/raspberry-pi](https://ubuntu.com/download/raspberry-pi)
and flash it to your microSD card using Raspberry Pi Imager.

Enable SSH, set hostname, and boot. Then:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release build-essential git
```

### 3.2 ROS 2 Humble Installation

```bash
# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
  python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 in every shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 Intel RealSense SDK

```bash
# Add Intel RealSense repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# ROS 2 wrapper
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

### 3.4 RTAB-Map

```bash
sudo apt install -y ros-humble-rtabmap-ros
```

### 3.5 MAVROS

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets (required by MAVROS)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 3.6 Additional Dependencies

```bash
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  python3-opencv \
  python3-numpy \
  python3-pip

pip3 install --user setuptools==58.2.0
```

### 3.7 Build the Workspace

```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws/src

# Clone this repository
git clone https://github.com/tejaswinisa1/rtabmap_slam_ros2 .

cd ~/uav_ws

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build (use limited parallelism on Pi 5 to avoid OOM)
colcon build --symlink-install \
  --packages-select uav_nodes uav_slam_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2

# Source the workspace
echo "source ~/uav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Dependency Installation

Complete dependency list with exact install commands:

```bash
# Core ROS 2
sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools

# SLAM
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-odom \
  ros-humble-rtabmap-sync \
  ros-humble-rtabmap-util \
  ros-humble-rtabmap-msgs

# Camera
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-image-transport \
  ros-humble-cv-bridge

# MAVROS / PX4
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs

# Python
pip3 install --user \
  numpy==1.24.4 \
  opencv-python==4.8.1.78 \
  transforms3d==0.4.1
```

---

## 5. RealSense D435 Setup

### 5.1 Verify Camera Hardware

```bash
# Plug in D435 via USB 3.0 (blue port)
rs-enumerate-devices

# Expected output includes:
#   Intel RealSense D435
#   Serial Number: XXXXXXXXXX
```

### 5.2 Launch Camera Driver

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  pointcloud.enable:=false
```

### 5.3 Verify Topics

```bash
ros2 topic list | grep camera
# Expected:
#   /camera/color/image_raw
#   /camera/color/camera_info
#   /camera/aligned_depth_to_color/image_raw
#   /camera/depth/image_rect_raw

ros2 topic hz /camera/color/image_raw
# Expected: ~15 Hz

ros2 topic hz /camera/aligned_depth_to_color/image_raw
# Expected: ~15 Hz
```

### 5.4 Visualise in RViz2

```bash
rviz2 -d ~/uav_ws/src/uav_slam_launch/config/uav_rviz.rviz
```

Add an **Image** display, set topic to `/camera/color/image_raw`.

---

## 6. RTAB-Map SLAM Testing

### 6.1 Indoor SLAM Test (SLAM only, no PX4)

```bash
ros2 launch uav_slam_launch slam_only.launch.py
```

Move the drone (or carry the camera) slowly around the room.

### 6.2 RViz Visualisation

```bash
rviz2 -d ~/uav_ws/src/uav_slam_launch/config/uav_rviz.rviz
```

Add displays:
- **Map** → `/map`
- **Odometry** → `/odom`
- **TF** (enable all frames)

### 6.3 Pose Verification

```bash
ros2 topic echo /odom --once
ros2 topic echo /rtabmap/localization_pose --once
```

### 6.4 TF Verification

```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing: map → odom → base_link → camera_link

ros2 run tf2_ros tf2_echo map base_link
```

---

## 7. PX4 SITL Testing

### 7.1 Install PX4 and Gazebo

```bash
# On a development machine (not Pi 5 — SITL is too heavy for Pi)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic_iris
```

### 7.2 Launch SITL + MAVROS + SLAM

```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris

# Terminal 2: SLAM + MAVROS bridge
ros2 launch uav_slam_launch px4_sitl.launch.py use_sim_time:=true
```

### 7.3 Verify MAVROS Connection

```bash
ros2 topic echo /mavros/state --once
# Expected: connected: True, armed: False, mode: "MANUAL"
```

### 7.4 Verify Vision Pose Integration

```bash
ros2 topic echo /mavros/vision_pose/pose --once
# Should show pose data flowing from SLAM to MAVROS
```

### 7.5 Enable EKF2 Vision Fusion in PX4

Set these PX4 parameters via QGroundControl or MAVLink:

```
EKF2_AID_MASK  = 24   (vision position + vision yaw)
EKF2_HGT_REF   = 3    (vision height)
EKF2_EV_DELAY  = 50   (ms, tune to your system latency)
EKF2_EV_NOISE_MD = 0
MAV_ODOM_LP    = 0
```

---

## 8. Drone Hardware Testing

Follow this progression strictly. Do not skip steps.

### Step 1 — Bench Test (motors off, props off)

```bash
# Verify all topics are publishing
ros2 topic list
ros2 topic hz /camera/color/image_raw
ros2 topic hz /odom
ros2 topic hz /mavros/vision_pose/pose

# Verify TF tree is complete
ros2 run tf2_tools view_frames
```

Expected: all topics at correct rates, TF tree shows `map → odom → base_link → camera_link`.

### Step 2 — Sensor Verification

```bash
# Move camera by hand, verify odometry responds
ros2 topic echo /odom

# Verify SLAM map builds
rviz2 -d ~/uav_ws/src/uav_slam_launch/config/uav_rviz.rviz
```

### Step 3 — Static SLAM Test (drone on ground, motors off)

```bash
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600
```

Verify:
- MAVROS connects to Cube Orange
- Vision pose flows to PX4
- PX4 EKF2 accepts vision data (check QGC estimator status)

### Step 4 — Hover Test (props on, low altitude)

- Arm in STABILIZED mode
- Hover at 1 m altitude for 30 seconds
- Verify position hold stability
- Monitor CPU usage: `htop`

### Step 5 — Slow Waypoint Test

```bash
# Launch full stack
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  arena_width:=10.0 arena_height:=10.0 altitude:=3.0 speed:=1.0

# Send first waypoint via MAVROS
ros2 topic pub /mavros/setpoint_position/local \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 3.0}}}" --once
```

### Step 6 — Full Autonomous Coverage Mission

```bash
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  arena_width:=50.0 arena_height:=50.0 \
  overlap:=0.2 altitude:=10.0 speed:=3.0 \
  fcu_url:=/dev/ttyACM0:921600
```

Monitor:
- `/uav/coverage_path` — lawnmower path
- `/uav/optimized_path` — smoothed path
- `/uav/filtered_detections` — deduplicated detections
- CPU usage should stay below 80%

---

## 9. Performance Optimization

### Why 640×480 Resolution?

Full 1080p would require ~4× more pixels to process per frame. At 640×480, RTAB-Map
feature extraction and matching runs comfortably on the Pi 5's ARM Cortex-A76 cores.
The D435 depth accuracy at this resolution is sufficient for SLAM at indoor/low-altitude
outdoor distances (0.3–10 m).

### Why 15 FPS?

The Pi 5 cannot sustain real-time SLAM at 30 fps without thermal throttling. At 15 fps,
the odometry node has ~66 ms per frame — enough for ORB feature extraction and matching
with 400 features. RTAB-Map's detection rate is further reduced to 1 Hz, meaning the
full loop-closure check runs only once per second.

### Why Dense Mapping is Disabled?

Dense point cloud generation (`Grid/3D=false`, `Grid/FromDepth=false`) is the single
largest CPU consumer in RTAB-Map. For UAV coverage missions, a 2D occupancy map or
pose graph is sufficient. Dense reconstruction can be done offline from the saved
database (`~/.ros/rtabmap_uav.db`).

### Expected CPU Usage

| Component | CPU (Pi 5, 4-core) |
|-----------|-------------------|
| realsense2_camera | ~8% |
| rgbd_odometry | ~25% |
| rtabmap (1 Hz) | ~15% |
| uav_nodes (all) | ~5% |
| MAVROS | ~3% |
| **Total** | **~56%** |

Peaks during loop closure may reach 75–80% briefly.

### Memory Optimization

- `Mem/STMSize=10` keeps only the 10 most recent nodes in short-term memory
- `Mem/IncrementalMemory=true` enables online map growth
- `OdomF2M/MaxSize=1000` caps the odometry feature map size
- RTAB-Map database is written to disk; RAM usage stays under 500 MB

### Thermal Management

```bash
# Monitor Pi 5 temperature
watch -n 2 vcgencmd measure_temp

# If throttling occurs (>80°C), add a heatsink/fan and reduce FPS:
# rgb_camera.profile:=640x480x10
```

---

## 10. Troubleshooting

### TF Issues

**Problem:** `map → odom` transform not publishing  
**Fix:** Ensure `rtabmap` node is running and has received at least one valid odometry message.
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /odom --once
```

**Problem:** `base_link → camera_link` missing  
**Fix:** The static TF publisher in `slam_only.launch.py` must be running.
```bash
ros2 run tf2_ros static_transform_publisher 0.05 0 0.02 0 0 0 base_link camera_link
```

**Problem:** TF extrapolation errors  
**Fix:** Enable `approx_sync:=true` in launch arguments. Check system clock sync.

---

### MAVROS Issues

**Problem:** MAVROS not connecting to Cube Orange  
**Fix:** Check serial port and baud rate.
```bash
ls /dev/ttyACM*   # or /dev/ttyUSB*
sudo chmod 666 /dev/ttyACM0
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600
```

**Problem:** `mavros/state` shows `connected: False`  
**Fix:** Verify PX4 MAVLink output is enabled on the correct UART. Set `MAV_0_CONFIG=101`
(TELEM1) in PX4 parameters.

**Problem:** Vision pose not accepted by PX4  
**Fix:** Verify `EKF2_AID_MASK` includes bit 3 (vision position). Check `EKF2_EV_DELAY`
matches your actual latency.

---

### RealSense Issues

**Problem:** Camera not detected  
**Fix:** Use USB 3.0 port (blue). Check with `rs-enumerate-devices`.

**Problem:** Depth and RGB not aligned  
**Fix:** Ensure `align_depth.enable:=true` in the camera launch.

**Problem:** Camera drops frames  
**Fix:** Reduce profile to `640x480x10`. Check USB bandwidth with `lsusb -t`.

**Problem:** `libusb` permission denied  
**Fix:**
```bash
sudo usermod -aG plugdev $USER
# Log out and back in
```

---

### PX4 EKF Issues

**Problem:** EKF2 not fusing vision data  
**Fix:** Check `EKF2_AID_MASK`. Verify vision pose covariance is reasonable (< 1.0).
Monitor `/mavros/local_position/pose` — it should match `/rtabmap/localization_pose`.

**Problem:** Position drift after takeoff  
**Fix:** Increase `EKF2_EV_DELAY` by 10 ms increments until drift stops.
Ensure SLAM is initialised before arming.

---

### RTAB-Map Crashes

**Problem:** RTAB-Map crashes with OOM  
**Fix:** Reduce `Mem/STMSize` to 5. Disable `Vis/MaxFeatures` to 200.
Add swap space:
```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**Problem:** RTAB-Map loop closure causes large jumps  
**Fix:** Increase `Vis/MinInliers` to 20. Set `RGBD/OptimizeFromGraphEnd=false`.

---

### High CPU Usage

**Problem:** CPU consistently above 90%  
**Fix (in order):**
1. Reduce camera FPS: `rgb_camera.profile:=640x480x10`
2. Reduce features: `Vis/MaxFeatures=200`
3. Reduce detection rate: `Rtabmap/DetectionRate=0.5`
4. Disable ORB detector node if not needed
5. Check for thermal throttling: `vcgencmd measure_temp`

---

## 11. UAV-Specific Design Notes

### Why Nav2 is Removed

Nav2 (Navigation 2) is designed for ground robots with differential drive or Ackermann
steering. It relies on costmaps, footprint inflation, and 2D obstacle avoidance — none
of which apply to a UAV that moves in 3D space and avoids obstacles by altitude.
Removing Nav2 saves ~200 MB RAM and eliminates several CPU-intensive costmap update
threads.

### Why Waypoint Control Instead of Nav2

PX4 handles all low-level stabilisation, attitude control, and position hold internally
via its EKF2 + flight controller. The companion computer only needs to send high-level
setpoints via MAVROS (`/mavros/setpoint_position/local`). This is simpler, more reliable,
and far more efficient than running a full Nav2 stack.

### Why Lightweight Mapping

For a coverage mission, the primary output is the mosaic image and detection log — not
a dense 3D map. RTAB-Map in lightweight mode provides:
- Accurate pose estimation for geo-referencing images
- Loop closure to correct drift over long missions
- A 2D occupancy map for basic situational awareness

Dense reconstruction can be performed offline from the saved `.db` file on a more
powerful machine.

### Why PX4 Handles Stabilisation

The Cube Orange + PX4 runs at 400 Hz for attitude control and 50 Hz for position control.
The Raspberry Pi 5 cannot match this real-time performance. The correct architecture is:
- **Pi 5**: perception, SLAM, path planning, mission logic (non-real-time)
- **Cube Orange**: stabilisation, attitude, motor control (hard real-time)

---

## 12. Package Structure

```
rtabmap_ros/
├── uav_slam_launch/              # UAV launch files and config
│   ├── launch/
│   │   ├── slam_only.launch.py       # D435 + RTAB-Map only
│   │   ├── slam_px4.launch.py        # SLAM + MAVROS + vision bridge
│   │   ├── full_uav_stack.launch.py  # Full mission stack
│   │   └── px4_sitl.launch.py        # SITL simulation
│   ├── params/
│   │   └── rtabmap_uav_params.yaml   # Pi 5-optimised RTAB-Map params
│   ├── config/
│   │   └── uav_rviz.rviz             # RViz2 configuration
│   ├── CMakeLists.txt
│   └── package.xml
│
├── uav_nodes/                    # Custom UAV Python nodes
│   ├── uav_nodes/
│   │   ├── lawnmower_planner.py      # Coverage path generator
│   │   ├── path_optimizer.py         # Chaikin path smoother
│   │   ├── orb_detector.py           # Lightweight ORB detection
│   │   ├── duplicate_filter.py       # Position-based dedup
│   │   └── vision_pose_bridge.py     # SLAM → MAVROS bridge
│   ├── setup.py
│   ├── setup.cfg
│   └── package.xml
│
├── rtabmap_slam/                 # RTAB-Map SLAM node (upstream, kept)
├── rtabmap_odom/                 # RTAB-Map odometry (upstream, kept)
├── rtabmap_sync/                 # Topic synchronisation (upstream, kept)
├── rtabmap_util/                 # Utility nodes (upstream, kept)
├── rtabmap_msgs/                 # Message definitions (upstream, kept)
├── rtabmap_conversions/          # Type conversions (upstream, kept)
│
└── [REMOVED - ground robot only]
    ├── rtabmap_costmap_plugins/  # Nav2 costmap — removed
    └── rtabmap_demos/            # Ground robot demos — not used
```

### Quick Launch Reference

```bash
# SLAM only (indoor testing, no PX4)
ros2 launch uav_slam_launch slam_only.launch.py

# SLAM + PX4 (hardware flight)
ros2 launch uav_slam_launch slam_px4.launch.py fcu_url:=/dev/ttyACM0:921600

# Full autonomous coverage mission
ros2 launch uav_slam_launch full_uav_stack.launch.py \
  arena_width:=50.0 arena_height:=50.0 altitude:=10.0

# PX4 SITL simulation
ros2 launch uav_slam_launch px4_sitl.launch.py
```

---

*This system is optimised for competition-ready UAV autonomous coverage missions.
All components are selected for minimal CPU overhead on Raspberry Pi 5 while
maintaining stable SLAM and reliable PX4 integration.*
