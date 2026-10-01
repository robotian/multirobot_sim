#!/bin/bash
# Only rmw_zenoh_cpp needs the system ROS 2; with Fast DDS the bundled libraries are used as before.
if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ]; then
    source /opt/ros/jazzy/setup.bash
fi
# SIM_MODE (docker-compose.yml, from .env): "stream" (default) = headless Kit app with WebRTC livestream;
# "headed" = the full desktop app in a window on the host's X display (needs scripts/x11_auth.sh and the
# X11 socket/DISPLAY that docker-compose.yml passes in).
if [ "${SIM_MODE:-stream}" = "headed" ]; then
    /isaac-sim/license.sh && /isaac-sim/privacy.sh && exec /isaac-sim/isaac-sim.sh "$@"
fi
exec /isaac-sim/runheadless.sh "$@"
