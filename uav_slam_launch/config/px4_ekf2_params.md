# PX4 EKF2 Parameters — RTAB-Map Visual SLAM Fusion
# Target: Cube Orange + PX4 v1.14+ + MAVROS + ROS2 Jazzy

Set these in QGroundControl → Vehicle Setup → Parameters.

## Required Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `EKF2_AID_MASK` | `24` | Vision position (bit 3) + vision yaw (bit 4) |
| `EKF2_HGT_REF` | `3` | Height reference: vision |
| `EKF2_EV_DELAY` | `50` | Vision delay ms — tune to your system |
| `EKF2_EV_NOISE_MD` | `0` | Use covariance from vision message |
| `EKF2_EV_POS_X` | `0.05` | Camera X offset from IMU (m) |
| `EKF2_EV_POS_Y` | `0.0` | Camera Y offset from IMU (m) |
| `EKF2_EV_POS_Z` | `-0.02` | Camera Z offset from IMU (m, negative = up) |
| `EKF2_EV_CTRL` | `15` | Enable all EV fusion bits |
| `MAV_ODOM_LP` | `0` | Disable odometry loopback |
| `COM_ARM_WO_GPS` | `1` | Allow arming without GPS |
| `EKF2_GPS_CTRL` | `0` | Disable GPS (vision only) |

## EKF2_AID_MASK Bit Reference

```
Bit 0 (1)  = GPS
Bit 1 (2)  = Optical flow
Bit 2 (4)  = Vision position
Bit 3 (8)  = Vision yaw
Bit 4 (16) = External vision (EV)
```

RTAB-Map only (no GPS): `EKF2_AID_MASK = 24` (bits 3+4)
RTAB-Map + GPS fusion:  `EKF2_AID_MASK = 25` (bits 0+3+4)

## Tuning EKF2_EV_DELAY

Measures latency from image capture to pose arriving at PX4.

Typical values for Raspberry Pi 5 + D435:
- @ 15 fps: 50–80 ms
- @ 10 fps: 60–90 ms

Measure actual delay:
```bash
ros2 topic echo /camera/color/image_raw --field header.stamp --once
ros2 topic echo /mavros/vision_pose/pose --field header.stamp --once
# Difference = EKF2_EV_DELAY
```

## Safe Vision-Only Flight Parameters

```
MPC_XY_VEL_MAX  = 2.0   # limit horizontal speed (m/s)
MPC_Z_VEL_MAX_UP = 1.0  # limit climb rate (m/s)
MPC_LAND_SPEED  = 0.5   # slow landing
MPC_THR_HOVER   = 0.35  # tune to your drone weight
```

## Verifying EKF2 Fusion in QGroundControl

1. Analyze → MAVLink Inspector → ESTIMATOR_STATUS
2. `pos_horiz_accuracy` < 0.5 m = good
3. `flags` bit 4 = 1 = EV position fused
