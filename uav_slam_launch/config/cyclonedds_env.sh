#!/usr/bin/env bash
# =============================================================================
# CycloneDDS Environment Setup for UAV SLAM — ROS2 Jazzy
# =============================================================================
# Source this file before launching any UAV SLAM nodes:
#   source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh
#
# Or add to ~/.bashrc for permanent effect:
#   echo "source ~/uav_ws/src/uav_slam_launch/config/cyclonedds_env.sh" >> ~/.bashrc
# =============================================================================

# Use CycloneDDS instead of FastDDS (default in Jazzy)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Point to the optimised CycloneDDS config
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_URI="file://${SCRIPT_DIR}/cyclonedds.xml"

# Increase DDS receive buffer (important for image topics on Pi 5)
export ROS_DOMAIN_ID=0

# Disable multicast for single-machine deployment (reduces network overhead)
export FASTRTPS_DEFAULT_PROFILES_FILE=""

echo "[CycloneDDS] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[CycloneDDS] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "[CycloneDDS] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
