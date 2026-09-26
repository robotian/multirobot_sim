#!/bin/bash
# Generate the A300 URDF (with the D435i) from robot.yaml.tmpl using Clearpath's own generator, and
# collect its meshes into sim/assets/a300/ for the Isaac Sim URDF importer.
# All robots share one configuration, so one URDF/USD serves the whole fleet.
set -euo pipefail
cd "$(dirname "$0")/.."
IMAGE="${ROBOT_IMAGE:-a300-robot:jazzy}"
mkdir -p sim/assets/a300

docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD/sim/assets/a300:/out" \
    -v "$PWD/scripts/flatten_urdf.py:/flatten_urdf.py:ro" \
    --entrypoint bash "$IMAGE" -c '
set -e
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/setup
sed -e "s/__NS__/a300_0000/g" -e "s/__SERIAL__/a300-0000/g" /opt/clearpath/robot.yaml.tmpl > /tmp/setup/robot.yaml
ros2 run clearpath_generator_common generate_description -s /tmp/setup
xacro /tmp/setup/robot.urdf.xacro -o /tmp/setup/robot.urdf
cp /tmp/setup/robot.yaml /out/robot.yaml
python3 /flatten_urdf.py /tmp/setup/robot.urdf /out a300.urdf
'
