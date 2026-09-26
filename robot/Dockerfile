# Robot-side container: stands in for the A300's onboard computer.
# ROS 2 Jazzy + Clearpath description stack + Foxglove bridge + GUI tools (rqt_image_view) + keyboard teleop (`camera_view`, `teleop` wrappers).
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-clearpath-config \
        ros-jazzy-clearpath-generator-common \
        ros-jazzy-clearpath-description \
        ros-jazzy-clearpath-platform-description \
        ros-jazzy-clearpath-sensors-description \
        ros-jazzy-clearpath-mounts-description \
        ros-jazzy-clearpath-control \
        ros-jazzy-realsense2-description \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-rqt-image-view \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-foxglove-bridge \
        gettext-base x11-apps \
    && rm -rf /var/lib/apt/lists/*

COPY config/robot.yaml.tmpl config/a300.rviz.tmpl /opt/clearpath/
COPY entrypoint.sh /entrypoint.sh
COPY bin/ /usr/local/bin/
RUN chmod +x /entrypoint.sh /usr/local/bin/teleop /usr/local/bin/camera_view /usr/local/bin/robot_state /usr/local/bin/rviz /usr/local/bin/foxglove

# `docker exec` shells bypass the entrypoint: source ROS for interactive (bashrc) and `bash -c` (BASH_ENV) shells.
RUN echo 'source /opt/ros/jazzy/setup.bash' > /etc/ros_env.sh \
    && echo 'source /etc/ros_env.sh' >> /etc/bash.bashrc
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1 \
    BASH_ENV=/etc/ros_env.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
