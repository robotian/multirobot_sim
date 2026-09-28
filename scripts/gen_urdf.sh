#!/bin/bash
# Generate the URDF (with the D435i) for every supported Clearpath model from its robot.<model>.yaml.tmpl using
# Clearpath's own generator, and collect each one's meshes into sim/assets/<model>/ for the Isaac Sim URDF
# importer. All robots of a given model share one configuration, so one URDF/USD per model serves the whole
# fleet regardless of how many robots run it (see ROBOT_MODEL_<i> in .env).
set -euo pipefail
cd "$(dirname "$0")/.."
IMAGE="${ROBOT_IMAGE:-clearpath-robot:jazzy}"
MODELS="a300 a200 j100 r100 j100_0921 j100_0936"
for m in $MODELS; do mkdir -p "sim/assets/$m"; done
# Middleware/domain written into robot.yaml only to satisfy clearpath_config's schema (the URDF depends on
# neither); FLEET_RMW from .env, like docker compose, and domain_id 0 (the actual per-robot value is filled in
# by entrypoint.sh at container start, not here).
RMW="${FLEET_RMW:-$(sed -n 's/^FLEET_RMW=//p' .env 2>/dev/null | tail -1)}"

docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -e "RMW=${RMW:-rmw_zenoh_cpp}" -e "MODELS=$MODELS" \
    -v "$PWD/sim/assets:/out" \
    -v "$PWD/scripts/flatten_urdf.py:/flatten_urdf.py:ro" \
    --entrypoint bash "$IMAGE" -c '
set -e
source /opt/ros/jazzy/setup.bash
for m in $MODELS; do
    rm -rf /tmp/setup && mkdir -p /tmp/setup
    # A real robot id (contains "_", e.g. j100_0921) already is "<model>-<unit>" once hyphenated -- clearpath_
    # config rejects anything else (see robot/entrypoint.sh for the full explanation); generic catalog models
    # get a throwaway "<model>-0000" serial instead, since this pass never runs a live per-slot robot.
    case "$m" in
        *_*) serial="${m//_/-}" ;;
        *) serial="${m}-0000" ;;
    esac
    sed -e "s/__NS__/${m}_0000/g" -e "s/__SERIAL__/${serial}/g" -e "s/__RMW__/$RMW/g" -e "s/__DOMAIN__/0/g" \
        "/opt/clearpath/robot.${m}.yaml.tmpl" > /tmp/setup/robot.yaml
    ros2 run clearpath_generator_common generate_description -s /tmp/setup
    xacro /tmp/setup/robot.urdf.xacro -o /tmp/setup/robot.urdf
    cp /tmp/setup/robot.yaml "/out/$m/robot.yaml"
    python3 /flatten_urdf.py /tmp/setup/robot.urdf "/out/$m" "$m.urdf"
done
'
