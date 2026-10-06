import os

import yaml

ROBOT_YAML = "/etc/clearpath/robot.yaml"


def robot_namespace():
    """$ROBOT_NAMESPACE (the sim containers set it), else robot.yaml's system.ros2.namespace (a real robot's shell
    doesn't set it), else none."""
    ns = os.environ.get("ROBOT_NAMESPACE")
    if ns:
        return ns
    try:
        with open(ROBOT_YAML) as f:
            config = yaml.safe_load(f) or {}
        return str(((config.get("system") or {}).get("ros2") or {}).get("namespace") or "")
    except (OSError, AttributeError, yaml.YAMLError):
        return ""
