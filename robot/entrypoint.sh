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
# A real robot with its own robot_data/<id>/robot.yaml (bind-mounted read-only, see docker-compose.yml) is used
# directly, unmodified -- no placeholder substitution needed at all, since its baked-in namespace/domain_id/
# middleware/serial_number/workspaces already match this project's own conventions exactly (confirmed by
# inspection: e.g. j100_0921's own robot.yaml already has namespace: j100_0921, domain_id: 0, middleware.
# implementation: rmw_fastrtps_cpp, workspaces: [/home/robot/colcon_ws/install/setup.bash]). It doesn't set
# system.localhost either, so clearpath_config's own default (this container's real OS hostname) is used --
# that's ROBOT_HOSTNAME_<i> from docker-compose.yml, cpr-<model>-<serial>, which is valid *and* happens to equal
# this robot's own system.hosts[0].hostname. A real robot id without its own robot_data folder (e.g. j100_0936,
# whose folder isn't present) falls back to the existing stripped-down .tmpl path below, same as ever.
if [[ "$ROBOT_MODEL" == *_* && -f "/robot_data/$ROBOT_MODEL/robot.yaml" ]]; then
    cp "/robot_data/$ROBOT_MODEL/robot.yaml" /etc/clearpath/robot.yaml
    # ...except the ROS domain: that belongs to the simulated network (ROS_DOMAIN_ID from .env, shared by the
    # sim and every robot container), not to one robot's real-world config. a200_0284's own yaml says
    # domain_id: 1, so its container sat on domain 1 while the sim was on domain 0 and it never saw a single sim
    # topic (j100_0921/a300_00036/... happen to use 0). Only the uncommented domain_id line is rewritten.
    fleet_domain="${ROS_DOMAIN_ID:-0}"
    real_domain="$(sed -n -E 's/^[[:space:]]*domain_id:[[:space:]]*([0-9]+).*/\1/p' /etc/clearpath/robot.yaml | head -1)"
    if [[ -n "$real_domain" && "$real_domain" != "$fleet_domain" ]]; then
        echo "[entrypoint] $ROBOT_MODEL: robot.yaml domain_id $real_domain -> $fleet_domain (the fleet's ROS_DOMAIN_ID)"
        sed -i -E "s/^([[:space:]]*domain_id:[[:space:]]*)[0-9]+/\1${fleet_domain}/" /etc/clearpath/robot.yaml
    fi
    # Same for the middleware (FLEET_RMW): generate_bash exports RMW_IMPLEMENTATION from it, and a robot on
    # another RMW than the sim sees nothing. The robot_data copies may also lag the real robots (now zenoh).
    fleet_rmw="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
    real_rmw="$(sed -n -E 's/^[[:space:]]*implementation:[[:space:]]*([a-z_]+).*/\1/p' /etc/clearpath/robot.yaml | head -1)"
    if [[ -n "$real_rmw" && "$real_rmw" != "$fleet_rmw" ]]; then
        echo "[entrypoint] $ROBOT_MODEL: robot.yaml middleware $real_rmw -> $fleet_rmw (the fleet's FLEET_RMW)"
        sed -i -E "s/^([[:space:]]*implementation:[[:space:]]*)[a-z_]+/\1${fleet_rmw}/" /etc/clearpath/robot.yaml
    fi
else
    sed -e "s/__NS__/${ROBOT_NAMESPACE}/g" -e "s/__SERIAL__/${ROBOT_SERIAL}/g" \
        -e "s/__RMW__/${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}/g" \
        -e "s/__DOMAIN__/${ROS_DOMAIN_ID:-0}/g" \
        "/opt/clearpath/robot.${ROBOT_MODEL}.yaml.tmpl" > /etc/clearpath/robot.yaml
fi

# robot.yaml's `workspaces` entry (see the template) needs its install dir to exist before anything can source
# it; colcon_build.sh/colcon build normally provide a real one, but this covers a workspace nobody has built
# yet (a fresh clone's empty colcon_ws) so setup.bash below never fails to source it.
mkdir -p /home/robot/colcon_ws/install
[ -f /home/robot/colcon_ws/install/setup.bash ] || echo '# nothing built in colcon_ws yet' > /home/robot/colcon_ws/install/setup.bash
# `robot` takes the uid/gid of the host user who owns the checkout, so the bind-mounted colcon_ws stays theirs
# whatever their uid (the image's 1000 only fits the typical single-user desktop). Read from ./scripts, which is
# mounted read-only and never chowned; HOST_UID/HOST_GID override. usermod skips it once done (docker restart).
host_uid="${HOST_UID:-$(stat -c %u /scripts 2>/dev/null || echo 1000)}"
host_gid="${HOST_GID:-$(stat -c %g /scripts 2>/dev/null || echo 1000)}"
if [[ "$host_uid" != 0 && ( "$(id -u robot)" != "$host_uid" || "$(id -g robot)" != "$host_gid" ) ]]; then
    groupmod -o -g "$host_gid" robot
    usermod -o -u "$host_uid" -g "$host_gid" robot
    echo "[entrypoint] robot user -> uid $host_uid gid $host_gid (owner of the host checkout)"
fi
chown -R robot:robot /home/robot/colcon_ws

source /opt/ros/jazzy/setup.bash  # only to make `ros2 run` available for the next line
ros2 run clearpath_generator_common generate_bash -s /etc/clearpath
source /etc/clearpath/setup.bash  # sources ROS, colcon_ws, and sets ROS_DOMAIN_ID/RMW_IMPLEMENTATION from robot.yaml
source /opt/clearpath_robot_ws/install/setup.bash  # overlay: clearpath_generator_robot + clearpath_sensors, built at image build time -- see Dockerfile
# One-shot, like generate_bash above: writes /etc/clearpath/{platform,manipulators,sensors}/config/*.yaml and
# .../launch/*.py (ros2_control, diagnostics, localization, teleop, twist_mux, per-sensor driver params, ...) so
# the real, unmodified Clearpath launch files this image already has installed (clearpath_control/
# clearpath_platform_description/clearpath_manipulators/clearpath_sensors) can be run against them later if
# wanted -- see robot/bin/generate_params and CLAUDE.md for what this can/can't produce.
generate_params
# One-shot: writes /etc/clearpath/robot.srdf (MoveIt semantic description) for real MoveIt launch files
# (mtu32_bringup's moveit.launch.py, etc.) to load -- see robot/bin/generate_srdf for why this isn't just the
# stock generate_semantic_description console_script.
generate_srdf
# ROS_NAMESPACE is only honoured by launch files; `ros2 run` tools need `--ros-args -r __ns:=` (see bin/teleop).
# (already written to /etc/robot_ns_env.sh above, for docker exec shells; this exports it for this process too.)
export ROS_NAMESPACE="${ROBOT_NAMESPACE}"
# Background service, like the robot's own systemd unit; restarted if it dies.
(while true; do robot_state || true; sleep 2; done) > /tmp/robot_state.log 2>&1 &
# The platform EKF (robot/bin/ekf): owns odom -> base_link, as on the real robot.
(while true; do ekf || true; sleep 2; done) > /tmp/ekf.log 2>&1 &
(while true; do foxglove || true; sleep 2; done) > /tmp/foxglove.log 2>&1 &
# Fake serial hardware for pruner_action_server (see robot/bin/pruner_stub) -- harmless on models that never
# launch it; restarted like the others so /dev/ttyOpenCR survives across a stub crash.
(while true; do pruner_stub || true; sleep 2; done) > /tmp/pruner_stub.log 2>&1 &

exec "$@"
