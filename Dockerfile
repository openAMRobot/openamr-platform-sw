FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install all dependencies in one layer
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pytest-cov \
    python3-vcstool \
    git \
    wget \
    curl \
    # Gazebo Harmonic + ROS 2 bridge
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    # Nav2 stack
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-msgs \
    ros-jazzy-opennav-docking \
    # SLAM
    ros-jazzy-slam-toolbox \
    # Laser scan filtering (scan_body_filter — clips rear arc before costmap)
    ros-jazzy-laser-filters \
    # AprilTag detection
    ros-jazzy-apriltag-ros \
    ros-jazzy-apriltag-msgs \
    # Robot description tools
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    # TF2
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    # Core ROS interfaces
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-ament-cmake-pytest \
    ros-jazzy-ament-lint-common \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    # Visualization
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy source and build
COPY ros2/src ./src

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Auto-source ROS and workspace in every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Entrypoint sources ROS before any CMD
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
