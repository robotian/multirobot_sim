#!/bin/bash
# Renders /etc/clearpath/robot.yaml for this robot (as on a real Clearpath robot), runs the same generator a
# real robot's systemd units run to turn it into /etc/clearpath/setup.bash, loads that, then execs the
# container command.
set -e
: "${ROBOT_NAMESPACE:?ROBOT_NAMESPACE must be set, e.g. a300_0000}"
: "${ROBOT_MODEL:=a300}"  # a300, a200, j100, r100, or a real robot's own id like j100_0921 -- see robot/config/robot.<model>.yaml.tmpl

# A real robot's own id doubles as its namespace directly (j100_0921, not j100_0921_0000): it's one specific
# physical robot with one fixed real identity, and the compose-provided ROBOT_NAMESPACE (<model>_<4-digit slot>,
# needed so multiple robots can share one *generic* model without colliding) is redundant/wrong for it -- a real
# robot's own ROS namespace on the actual hardware is just its serial, no slot concept. docker-compose.yml can't
# compute this conditionally (its interpolation has no way to inspect ROBOT_MODEL's content, confirmed by
# testing a bash-style substitution there directly -- "invalid interpolation format"), so it's done here
# instead, matching the ROBOT_SERIAL fix just below. This reassigns ROBOT_NAMESPACE itself (not a new variable)
# so everything downstream in this script (including ROS_NAMESPACE below, which is always the same value) picks
# it up automatically. /etc/robot_ns_env.sh persists the same override for `docker exec` shells -- bin/teleop,
# camera_view, rviz, all of which just read $ROBOT_NAMESPACE, started fresh from the compose container's own
# (still slot-suffixed) base environment, not from anything this script's own process exports -- written fresh
# (not appended) on every entrypoint run, including a plain `docker restart`, so repeated restarts of the same
# container never accumulate duplicate lines.
if [[ "$ROBOT_MODEL" == *_* ]]; then
    ROBOT_NAMESPACE="$ROBOT_MODEL"
fi
export ROBOT_NAMESPACE
printf 'export ROBOT_NAMESPACE=%q\nexport ROS_NAMESPACE=%q\n' "$ROBOT_NAMESPACE" "$ROBOT_NAMESPACE" > /etc/robot_ns_env.sh

# Clearpath hostnames/serials cannot contain '_', and clearpath_config's SerialNumber parser requires exactly
# "<model>-<decimal unit>" (2 fields) or "cpr-<model>-<decimal unit>" (3, first field literally "cpr") -- no
# other shape is accepted. Generic catalog models (ROBOT_MODEL="a300") get a synthetic per-slot serial from the
# container's own namespace (a300_0000 -> a300-0000), since multiple robots can share one generic model and
# need distinct serials. A real robot's own id (ROBOT_MODEL="j100_0921") already *is* "<model>-<unit>" once its
# underscore becomes a hyphen, and is used directly instead -- it has one fixed real serial regardless of which
# fleet slot it's spawned in, so deriving from ROBOT_NAMESPACE would wrongly append that slot's index too
# (e.g. namespace j100_0921_0001 -> j100-0921-0001, 3 non-"cpr" fields, which the parser rejects outright).
if [[ "$ROBOT_MODEL" == *_* ]]; then
    export ROBOT_SERIAL="${ROBOT_MODEL//_/-}"
else
    export ROBOT_SERIAL="${ROBOT_NAMESPACE//_/-}"
fi
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
# (already written to /etc/robot_ns_env.sh above, for docker exec shells; this exports it for this process too.)
export ROS_NAMESPACE="${ROBOT_NAMESPACE}"
# Background service, like the robot's own systemd unit; restarted if it dies.
(while true; do robot_state || true; sleep 2; done) > /tmp/robot_state.log 2>&1 &
(while true; do foxglove || true; sleep 2; done) > /tmp/foxglove.log 2>&1 &

exec "$@"
