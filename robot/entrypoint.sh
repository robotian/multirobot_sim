#!/bin/bash
# Renders /etc/clearpath/robot.yaml for this robot (as on a real Clearpath robot),
# sources ROS, then execs the container command.
set -e
: "${ROBOT_NAMESPACE:?ROBOT_NAMESPACE must be set, e.g. a300_0000}"

# Clearpath hostnames/serials cannot contain '_' -> a300_0000 becomes a300-0000
export ROBOT_SERIAL="${ROBOT_NAMESPACE//_/-}"
mkdir -p /etc/clearpath
sed -e "s/__NS__/${ROBOT_NAMESPACE}/g" -e "s/__SERIAL__/${ROBOT_SERIAL}/g" \
    /opt/clearpath/robot.yaml.tmpl > /etc/clearpath/robot.yaml

source /opt/ros/jazzy/setup.bash
# ROS_NAMESPACE is only honoured by launch files; `ros2 run` tools need `--ros-args -r __ns:=` (see bin/teleop).
export ROS_NAMESPACE="${ROBOT_NAMESPACE}"
# Background service, like the robot's own systemd unit; restarted if it dies.
(while true; do robot_state || true; sleep 2; done) > /tmp/robot_state.log 2>&1 &
(while true; do foxglove || true; sleep 2; done) > /tmp/foxglove.log 2>&1 &

exec "$@"
