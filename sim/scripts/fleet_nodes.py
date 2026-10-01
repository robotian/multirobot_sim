"""rclpy nodes created inside OmniGraph ScriptNodes (setup_scene.py's CMD_VEL/GPS/LIDAR2D/LIDAR3D scripts), by robot
namespace. Deleting a robot's graph (a respawn, see setup_scene.spawn_fleet) does not run the ScriptNodes' cleanup()
-- seen live: the removed robot's cmd_vel subscriber node stayed in the ROS graph -- so spawn_fleet destroys them
here instead. Shared through the module cache: the scripts and setup_scene.py run in the same interpreter."""
_states = {}


def track(ns, state):
    """Register a ScriptNode's per_instance_state whose .node (and optional .executor) belong to robot `ns`."""
    _states.setdefault(str(ns), []).append(state)


def destroy(ns):
    """Destroy robot `ns`'s nodes; sets state.node = None, so a late cleanup() of the script is a no-op."""
    n = 0
    for state in _states.pop(str(ns), []):
        node = getattr(state, "node", None)
        if node is None:
            continue
        try:
            if getattr(state, "executor", None) is not None:
                state.executor.shutdown()
            node.destroy_node()
            n += 1
        except Exception as e:
            print(f"[fleet] destroying {ns}'s rclpy node failed: {e}", flush=True)
        state.node = None
    return n
