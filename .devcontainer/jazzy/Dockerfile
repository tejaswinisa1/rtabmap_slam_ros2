FROM osrf/ros:jazzy-desktop-full

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # ROS2 Jazzy UAV stack
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
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-setuptools \
    python3-pip \
    # Python 3.12 packages
    python3-opencv \
    python3-numpy \
    # Utilities
    git curl wget htop \
    && /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# ── Create vscode user ────────────────────────────────────────────────────────
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo,dialout,plugdev ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# ── Workspace setup ───────────────────────────────────────────────────────────
RUN mkdir -p /home/${USERNAME}/uav_ws/src && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/uav_ws

# ── Shell environment ─────────────────────────────────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/uav_ws
