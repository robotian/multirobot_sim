# SubTask.msg
# Represents a sub-task that is part of a larger task.
# Each sub-task can be a motion, operation, or other activity.

# Enum values for sub-task types
int32 MOVING = 1
int32 HARVESTING = 2
int32 DOCKING = 3
int32 CHARGING = 4
int32 LOADING = 5
int32 UNLOADING = 6
int32 UNDOCKING = 7

int32 sub_task_id       # Unique identifier for the sub-task
int32 type              # Type of the sub-task (choose from constants above)
string description      # Brief description of the sub-task
WayPoint[] data         # Sequence of waypoints for navigation (if applicable)
string data_str         # Optional string data (alternative to waypoints)

DockGoal dock_goal      # Docking goal details if this is a docking sub-task
UndockGoal undock_goal  # Undocking goal details if this is an undocking sub-task