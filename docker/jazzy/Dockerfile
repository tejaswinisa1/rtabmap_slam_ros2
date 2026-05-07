FROM osrf/ros:jazzy-desktop-full

# UAV SLAM Docker image — ROS2 Jazzy / Ubuntu 24.04
# Raspberry Pi 5 + Intel RealSense D435 + PX4 + MAVROS + CycloneDDS

RUN apt-get update && apt-get install -y \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-realsense2-camera \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-mavros-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-cyclonedds \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-setuptools \
    python3-opencv \
    python3-numpy \
    git curl wget htop \
    && /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Set CycloneDDS as default RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc

WORKDIR /uav_ws
