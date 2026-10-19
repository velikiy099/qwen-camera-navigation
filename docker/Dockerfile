# Native architecture by default: arm64 on Apple Silicon, amd64 on Intel/AMD.
# The lecture-aligned ROS/Gazebo pair is Humble + Fortress (NOT Gazebo Classic).
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress ros-humble-ros-gz ros-humble-rclpy ros-humble-std-msgs ros-humble-std-srvs \
    ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-rosgraph-msgs \
    ros-humble-ros2launch ros-humble-launch-ros ros-humble-ament-index-python \
    ros-humble-rqt-image-view ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions python3-pytest python3-yaml python3-pil \
    xvfb x11vnc novnc websockify openbox xterm dbus-x11 x11-utils \
    mesa-utils libgl1-mesa-dri libglx-mesa0 fonts-dejavu-core procps tini \
    && rm -rf /var/lib/apt/lists/*
ENV DISPLAY=:1 QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
    XDG_RUNTIME_DIR=/tmp/runtime-pai ROS_LOCALHOST_ONLY=1 IGN_PARTITION=pai_visual_robot_v3 \
    PAI_LOG_DIR=/workspace/logs
WORKDIR /workspace
COPY src /workspace/src
COPY tools /workspace/tools
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select pai_robot \
    && PYTHONPATH=/workspace/src/pai_robot python3 -m unittest discover -s src/pai_robot/test -v
COPY docker/entrypoint.sh /usr/local/bin/pai-entrypoint
RUN chmod +x /usr/local/bin/pai-entrypoint \
    && echo 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; cd /workspace' >> /root/.bashrc
EXPOSE 80
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/pai-entrypoint"]
