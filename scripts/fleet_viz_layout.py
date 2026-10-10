#!/usr/bin/env python3
"""Foxglove layout for the fleet view (scripts/fleet_viz.py): every robot in one 3D panel.

Reads the relay's status (its robots, their relayed topics and frames; JSON from a file or stdin) and writes a
layout to import in Foxglove (Layouts > Import from file), connected to the fleet bridge
(ws://<host>:FLEET_VIZ_PORT):

- 3D panel, display frame ref_frame (the shared world frame), one URDF layer per robot from
  /fleet/<ns>/robot_description with frame prefix "<ns>/" (the relay's renamed frames);
- each robot's plan, MPPI trajectory, footprint and 2D scan in its own colour, one robot's map (all robots
  load the same map; the others are listed, hidden), local costmaps hidden;
- only each robot's base_link axes shown (a robot has 40-120 frames);
- --cameras: an Image panel per robot (straight from the robot's camera topic: raw images are heavy, so
  only when asked).

Regenerate after the fleet changes (other robots): the URDF layers are per robot.
"""
import argparse
import json
import sys

COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4", "#f032e6", "#9a6324"]
CAMERA = "sensors/camera_0/color/image"


def layout(status, cameras=False, map_robot=None):
    robots = list(status["robots"])
    if not robots:
        raise SystemExit("no robots in the status: is scripts/fleet_viz.sh running, with robots up?")
    map_robot = map_robot or robots[0]
    shared = status.get("shared_frames", ["ref_frame"])
    topics, layers, transforms = {}, {}, {}
    layers["grid"] = {"layerId": "foxglove.Grid", "instanceId": "grid", "label": "Grid", "visible": True,
                      "frameLocked": True, "frameId": shared[0], "size": 100,
                      "divisions": 50, "lineWidth": 1, "color": "#80808060", "position": [0, 0, 0],
                      "rotation": [0, 0, 0], "order": 1}
    for i, ns in enumerate(robots):
        c = COLORS[i % len(COLORS)]
        info = status["robots"][ns]
        rel = {t[len(f"/fleet/{ns}/"):]: t for t in info["topics"]}
        layers[f"urdf-{ns}"] = {"layerId": "foxglove.Urdf", "instanceId": f"urdf-{ns}", "label": ns,
                                "visible": True, "frameLocked": True, "sourceType": "topic",
                                "topic": f"/fleet/{ns}/robot_description", "url": "", "filePath": "",
                                "parameter": "", "framePrefix": f"{ns}/", "displayMode": "visual",
                                "fallbackColor": c, "order": 2 + i}
        for f in info["frames"]:
            if f not in shared:
                transforms[f"frame:{f}"] = {"visible": f == f"{ns}/base_link"}
        for name, t in rel.items():
            if name in ("plan", "optimal_trajectory"):
                width = 0.08 if name == "plan" else 0.04
                topics[t] = {"visible": True, "lineWidth": width, "gradient": [c, c]}
            elif name.endswith("published_footprint"):
                topics[t] = {"visible": True, "color": c, "lineWidth": 0.04}
            elif name.startswith("sensors/") and "scan" in name:
                # scan_filtered when the robot has it, else the raw scan
                filtered = name.endswith("scan_filtered") or not any(n.endswith("scan_filtered") for n in rel)
                topics[t] = {"visible": filtered, "colorMode": "flat", "flatColor": c, "pointSize": 3}
            elif name == "map":
                topics[t] = {"visible": ns == map_robot, "colorMode": "map", "alpha": 0.6}
            elif name.endswith("costmap"):
                topics[t] = {"visible": False, "colorMode": "costmap", "alpha": 0.4}
            elif name == "robot_description":
                topics[t] = {"visible": False}
    three_d = {
        "cameraState": {"perspective": True, "distance": 45, "phi": 40, "thetaOffset": 45, "fovy": 45,
                        "near": 0.5, "far": 5000, "target": [0, 0, 0], "targetOffset": [0, 0, 0],
                        "targetOrientation": [0, 0, 0, 1]},
        "followMode": "follow-none", "followTf": shared[0],
        # labels on: only each robot's base_link is shown, so they name the robots
        "scene": {"transforms": {"showLabel": True, "labelSize": 0.25, "axisScale": 0.6},
                  "ignoreColladaUpAxis": True},
        "transforms": transforms, "topics": topics, "layers": layers,
        "publish": {"type": "point", "poseTopic": "", "pointTopic": "", "poseEstimateTopic": ""},
        "imageMode": {},
    }
    config = {"3D!fleet": three_d}
    tree = "3D!fleet"
    if cameras:
        images = []
        for ns in robots:
            pid = f"Image!{ns}"
            config[pid] = {"imageMode": {"imageTopic": f"/{ns}/{CAMERA}"}, "cameraState": {}, "followMode":
                           "follow-pose", "scene": {}, "transforms": {}, "topics": {}, "layers": {}, "publish": {}}
            images.append(pid)
        column = images[-1]
        for pid in reversed(images[:-1]):
            column = {"direction": "column", "first": pid, "second": column}
        tree = {"direction": "row", "first": "3D!fleet", "second": column, "splitPercentage": 70}
    return {"configById": config, "globalVariables": {}, "userNodes": {}, "playbackConfig": {"speed": 1},
            "layout": tree}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("status", nargs="?", default="-", help="the relay's status JSON (default: stdin)")
    ap.add_argument("-o", "--output", default="-", help="layout file (default: stdout)")
    ap.add_argument("--cameras", action="store_true", help="an Image panel per robot")
    ap.add_argument("--map-robot", help="the robot whose map is shown (default: the first)")
    a = ap.parse_args()
    status = json.load(sys.stdin if a.status == "-" else open(a.status))
    text = json.dumps(layout(status, a.cameras, a.map_robot), indent=1)
    if a.output == "-":
        print(text)
    else:
        with open(a.output, "w") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
