#!/bin/bash
# Renders /etc/clearpath/robot.yaml for this robot (as on a real Clearpath robot), runs the same generator a
# real robot's systemd units run to turn it into /etc/clearpath/setup.bash, loads that, then execs the
# container command.
set -e
: "${ROBOT_NAMESPACE:?ROBOT_NAMESPACE must be set, e.g. a300_0000}"
: "${ROBOT_MODEL:=a300}"  # a300, a200, j100 (Jackal) or r100 (Ridgeback) -- see robot/config/robot.<model>.yaml.tmpl

# Clearpath hostnames/serials cannot contain '_' -> a300_0000 becomes a300-0000
export ROBOT_SERIAL="${ROBOT_NAMESPACE//_/-}"
mkdir -p /etc/clearpath
sed -e "s/__NS__/${ROBOT_NAMESPACE}/g" -e "s/__SERIAL__/${ROBOT_SERIAL}/g" \
    -e "s/__RMW__/${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}/g" \
    -e "s/__DOMAIN__/${ROS_DOMAIN_ID:-0}/g" \
    "/opt/clearpath/robot.${ROBOT_MODEL}.yaml.tmpl" > /etc/clearpath/robot.yaml

# robot.yaml's `workspaces` entry (see the template) needs its install dir to exist before anything can source
# it; colcon_build.sh/colcon build normally provide a real one, but this covers a workspace nobody has built
# yet (a fresh clone's empty colcon_ws) so setup.bash below never fails to source it.
mkdir -p /home/robot/colcon_ws/install
[ -f /home/robot/colcon_ws/install/setup.bash ] || echo '# nothing built in colcon_ws yet' > /home/robot/colcon_ws/install/setup.bash
chown -R robot:robot /home/robot/colcon_ws

source /opt/ros/jazzy/setup.bash  # only to make `ros2 run` available for the next line
ros2 run clearpath_generator_common generate_bash -s /etc/clearpath
source /etc/clearpath/setup.bash  # sources ROS, colcon_ws, and sets ROS_DOMAIN_ID/RMW_IMPLEMENTATION from robot.yaml
# ROS_NAMESPACE is only honoured by launch files; `ros2 run` tools need `--ros-args -r __ns:=` (see bin/teleop).
export ROS_NAMESPACE="${ROBOT_NAMESPACE}"
# Background service, like the robot's own systemd unit; restarted if it dies.
(while true; do robot_state || true; sleep 2; done) > /tmp/robot_state.log 2>&1 &
(while true; do foxglove || true; sleep 2; done) > /tmp/foxglove.log 2>&1 &

exec "$@"
