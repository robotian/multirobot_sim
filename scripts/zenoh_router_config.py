#!/usr/bin/env python3
"""Write a real robot's zenoh router config: rmw_zenoh_cpp's default + rate limits on what leaves over WiFi.

Runs on the robot (plain python3, no ROS needed), e.g. from the host:
  ssh robot@<ip> python3 - <wifi_if> < scripts/zenoh_router_config.py
writes ~/zenoh_config/router.json5; then point robot.yaml at it (system.ros2.middleware.profile), which makes
Clearpath's generated /etc/clearpath/zenoh-router-start export ZENOH_ROUTER_CONFIG_URI. See scripts/CLAUDE.md.

Only egress on <wifi_if> (default wlp3s0) is limited: the robot's own nodes talk to their router over loopback and
keep every message. Keys are <domain>/<namespace>/<topic>/<type>/<hash>, so */* matches any domain and namespace.
"""
import os
import sys

DEFAULT = '/opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5'
OUT = os.path.expanduser('~/zenoh_config/router.json5')
iface = sys.argv[1] if len(sys.argv) > 1 else 'wlp3s0'

# (topic under the robot's namespace, max Hz off the robot)
RULES = [
    # ros2_control diagnostics: ~1 kHz each on a300_00036 (~10 MB/s together); nothing off the robot needs them fast
    ('manipulators/controller_manager/introspection_data/full', 1.0),
    ('manipulators/controller_manager/introspection_data/values', 1.0),
    ('manipulators/controller_manager/introspection_data/names', 1.0),
    ('manipulators/controller_manager/statistics/full', 1.0),
    ('manipulators/controller_manager/statistics/values', 1.0),
    ('manipulators/controller_manager/statistics/names', 1.0),
    ('manipulators/arm_0_joint_trajectory_controller/controller_state', 10.0),
    # cameras: compressed color is ~2.5 MB/s each at 30 fps
    ('sensors/camera_0/color/compressed', 10.0),
    ('sensors/camera_1/color/compressed', 10.0),
]

t = open(DEFAULT).read()
anchor = '  // /// The downsampling declaration.\n'
assert t.count(anchor) == 1, f'{DEFAULT}: downsampling comment not found once (rmw_zenoh_cpp changed?)'
assert '\n  downsampling:' not in t, f'{DEFAULT} already has an active downsampling block'
rules = ''.join(f'        {{ key_expr: "*/*/{topic}/**", freq: {hz} }},\n' for topic, hz in RULES)
block = (
    '  /// Rate limits on what leaves this robot over WiFi, written by multirobot_sim scripts/zenoh_router_config.py\n'
    '  downsampling: [\n'
    '    {\n'
    '      id: "wifi_egress",\n'
    f'      interfaces: [ "{iface}" ],\n'
    '      flows: [ "egress" ],\n'
    '      messages: [ "put" ],\n'
    '      rules: [\n'
    f'{rules}'
    '      ],\n'
    '    },\n'
    '  ],\n\n')
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, 'w') as f:
    f.write(t.replace(anchor, block + anchor))
print(f'wrote {OUT} (WiFi interface {iface}, {len(RULES)} rules)')
