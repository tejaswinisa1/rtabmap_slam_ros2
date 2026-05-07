# UAV SLAM Docker — ROS2 Jazzy

Docker image for the UAV SLAM stack: Raspberry Pi 5 + D435 + PX4 + MAVROS.

## Build

```bash
docker build -t uav_slam_jazzy:latest -f docker/jazzy/Dockerfile .
```

## Run (with display + USB access)

```bash
xhost +local:docker

docker run -it --rm \
  --privileged \
  --network=host \
  --env=DISPLAY \
  --env=QT_X11_NO_MITSHM=1 \
  --env=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/bus/usb \
  uav_slam_jazzy:latest \
  bash
```

## Inside container

```bash
source /opt/ros/jazzy/setup.bash

# Clone and build UAV packages
mkdir -p /uav_ws/src && cd /uav_ws/src
git clone https://github.com/tejaswinisa1/rtabmap_slam_ros2 .
cd /uav_ws
colcon build --packages-select uav_nodes uav_slam_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 \
  --parallel-workers 2
source install/setup.bash

# Launch SLAM
ros2 launch uav_slam_launch slam_only.launch.py
```
