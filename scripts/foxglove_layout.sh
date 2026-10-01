#!/bin/bash
# Write a Foxglove layout per robot: foxglove/<ns>.json, a 3D panel showing that robot's model.
# The 3D panel only loads a URDF on its own from /robot_description; ours is /<ns>/robot_description, so the
# layout adds a URDF custom layer reading that topic. Meshes (package://...) are fetched through the robot's
# foxglove_bridge (its "assets" capability), TF comes from /<ns>/tf(_static), which the panel picks up by schema.
# In Foxglove: connect to ws://<host>:<8765 + slot>, then Layouts -> Import from file -> foxglove/<ns>.json.
#   scripts/foxglove_layout.sh              # every running robot container
#   scripts/foxglove_layout.sh j100_0921    # given namespaces
set -euo pipefail
cd "$(dirname "$0")/.."

if [ $# -gt 0 ]; then
    namespaces=("$@")
else
    # container name == ROS namespace (generic <model>_%04d, or a real robot's own id)
    mapfile -t namespaces < <(docker ps --format '{{.Names}}' | grep -E '^[a-z0-9]+_[0-9]{4,}$' | sort)
fi
[ ${#namespaces[@]} -gt 0 ] || { echo "no running robot containers; pass namespaces as arguments" >&2; exit 1; }

mkdir -p foxglove
for ns in "${namespaces[@]}"; do
    python3 - "$ns" > "foxglove/$ns.json" <<'EOF'
import json, sys
ns = sys.argv[1]
panel = "3D!robot"
layout = {
    "configById": {
        panel: {
            # display frame = the robot, camera follows its position but not its heading
            "followTf": "base_link",
            "followMode": "follow-position",
            "scene": {"transforms": {"showLabel": False}, "ignoreColladaUpAxis": True},
            "transforms": {},
            "topics": {},
            "layers": {
                "grid": {
                    "layerId": "foxglove.Grid", "instanceId": "grid", "label": "Grid", "visible": True,
                    "frameLocked": True, "frameId": "odom", "size": 20, "divisions": 20, "lineWidth": 1,
                    "color": "#248eff", "position": [0, 0, 0], "rotation": [0, 0, 0],
                },
                "robot": {
                    "layerId": "foxglove.Urdf", "instanceId": "robot", "label": ns, "visible": True,
                    "frameLocked": True, "sourceType": "topic", "topic": f"/{ns}/robot_description",
                    "url": "", "filePath": "", "parameter": "", "framePrefix": "",
                    "displayMode": "visual", "fallbackColor": "#ffffff",
                },
            },
            "cameraState": {
                "perspective": True, "distance": 4, "phi": 60, "thetaOffset": 45,
                "targetOffset": [0, 0, 0], "target": [0, 0, 0], "targetOrientation": [0, 0, 0, 1],
                "fovy": 45, "near": 0.01, "far": 5000,
            },
        },
    },
    "globalVariables": {},
    "userNodes": {},
    "playbackConfig": {"speed": 1},
    "layout": panel,
}
print(json.dumps(layout, indent=2))
EOF
    echo "foxglove/$ns.json"
done
