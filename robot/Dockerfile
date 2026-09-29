# Robot-side container: stands in for a Clearpath robot's onboard computer. One image serves every supported
# model (A300, A200, Jackal/j100, Ridgeback/r100, plus real MTU robots j100_0921/j100_0936 -- see ROBOT_MODEL
# in docker-compose.yml) -- they share the same ROS packages, and only robot.yaml (rendered from the matching
# robot/config/robot.<model>.yaml.tmpl) differs. clearpath-manipulators(-description)/kortex-description are
# only needed by the real MTU robots' Kinova arm+gripper, listed explicitly rather than left as an incidental
# transitive dependency of clearpath-control, since they're now something this project actually relies on.
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
        ros-jazzy-clearpath-manipulators \
        ros-jazzy-clearpath-manipulators-description \
        ros-jazzy-kortex-description \
        ros-jazzy-clearpath-control \
        ros-jazzy-realsense2-description \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-rqt-image-view \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rmw-zenoh-cpp \
        gettext-base x11-apps \
    && rm -rf /var/lib/apt/lists/*

# clearpath_generator_robot + clearpath_sensors: the real per-robot param/launch generator a physical Clearpath
# robot runs (github.com/clearpathrobotics/clearpath_robot, jazzy branch) -- NOT packaged in the public Jazzy apt
# repo (only clearpath_generator_common is, via ros-jazzy-clearpath-generator-common above), so it's built from
# source here instead. clearpath_generator_common's own ParamGenerator/LaunchGenerator leave generate_sensors()
# (and, for LaunchGenerator, every method) abstract/NotImplementedError -- RobotParamGenerator/RobotLaunchGenerator
# are the real subclasses that implement them. clearpath_sensors is a second, separate package (default per-
# sensor-type param/launch templates, e.g. the ZED2i/microstrain-IMU ones RobotParamGenerator's generate_sensors()
# merges real robot.yaml overrides into) that RobotParamGenerator/RobotLaunchGenerator read via
# get_package_share_directory('clearpath_sensors') at generation time, so both packages are needed, not just
# clearpath_generator_robot. Only these two of the repo's six packages are built: the other four
# (clearpath_robot metapackage, clearpath_hardware_interfaces, clearpath_motor_drivers, clearpath_tests) are for
# real motor-controller/CAN hardware this sim has no equivalent of. Both packages' package.xml exec_depends list
# many genuinely-unavailable hardware/sensor-driver packages (clearpath_hardware_interfaces, micro_ros_agent,
# realsense2_camera, velodyne_driver, ...) -- confirmed harmless: colcon build never checks exec_depends, and the
# actual code here only imports clearpath_config/clearpath_generator_common/its own modules, never those
# packages' own Python (confirmed by successfully building and running this standalone before adding it here).
RUN git clone --depth 1 -b jazzy https://github.com/clearpathrobotics/clearpath_robot.git /tmp/clearpath_robot_src \
    && mkdir -p /opt/clearpath_robot_ws/src \
    && cp -r /tmp/clearpath_robot_src/clearpath_generator_robot /tmp/clearpath_robot_src/clearpath_sensors /opt/clearpath_robot_ws/src/ \
    && rm -rf /tmp/clearpath_robot_src
RUN bash -c "source /opt/ros/jazzy/setup.bash && cd /opt/clearpath_robot_ws && colcon build" \
    && rm -rf /opt/clearpath_robot_ws/build /opt/clearpath_robot_ws/log

COPY config/robot.a300.yaml.tmpl config/robot.a200.yaml.tmpl config/robot.j100.yaml.tmpl config/robot.r100.yaml.tmpl config/robot.j100_0936.yaml.tmpl config/robot.rviz.tmpl /opt/clearpath/
COPY entrypoint.sh /entrypoint.sh
COPY bin/ /usr/local/bin/
RUN chmod +x /entrypoint.sh /usr/local/bin/teleop /usr/local/bin/camera_view /usr/local/bin/robot_state /usr/local/bin/rviz /usr/local/bin/foxglove /usr/local/bin/generate_params

# ROS workspace user. Ubuntu 24.04 (this image's base) already ships a uid/gid 1000 "ubuntu" account, which is
# renamed rather than adding a second uid 1000 user; 1000 is also the typical host user's uid, so files the
# bind-mounted ./colcon_ws (below) are readable/writable from both sides without extra chown juggling.
RUN usermod -l robot -d /home/robot -m ubuntu && groupmod -n robot ubuntu \
    && mkdir -p /home/robot/colcon_ws/src && chown -R robot:robot /home/robot
# Real content comes from the ./colcon_ws bind mount (docker-compose.yml), shared by every robot container, so
# an edit made in one robot's container is visible in all of them (and on the host). Container processes still
# run as root by default (entrypoint.sh needs it); use `docker exec -u robot -it <robot> bash` to build/edit as
# the workspace owner.

# `docker exec` shells bypass the entrypoint: source ROS for interactive (bashrc) and `bash -c` (BASH_ENV)
# shells. Prefer the real /etc/clearpath/setup.bash the entrypoint generates from robot.yaml (ROS + colcon_ws +
# ROS_DOMAIN_ID/RMW_IMPLEMENTATION) once it exists; fall back to plain ROS for the brief window before it does.
# Then, if present, /etc/robot_ns_env.sh -- written fresh by entrypoint.sh on every start, it overrides
# ROBOT_NAMESPACE/ROS_NAMESPACE for a real robot (ROBOT_MODEL like j100_0921) to that robot's own id instead of
# the compose container's slot-suffixed base environment (j100_0921_0000); sourced last so it wins.
RUN printf '%s\n' \
        'if [ -f /etc/clearpath/setup.bash ]; then' \
        '    source /etc/clearpath/setup.bash' \
        'else' \
        '    source /opt/ros/jazzy/setup.bash' \
        'fi' \
        'source /opt/clearpath_robot_ws/install/setup.bash' \
        '[ -f /etc/robot_ns_env.sh ] && source /etc/robot_ns_env.sh' > /etc/ros_env.sh \
    && echo 'source /etc/ros_env.sh' >> /etc/bash.bashrc
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1 \
    BASH_ENV=/etc/ros_env.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
