#!/bin/bash
# Only rmw_zenoh_cpp needs the system ROS 2; with Fast DDS the bundled libraries are used as before.
if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ]; then
    source /opt/ros/jazzy/setup.bash
fi
exec /isaac-sim/runheadless.sh "$@"
