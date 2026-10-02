# Task.msg
# Represents a high-level task assigned to a robot, which may include multiple sub-tasks.

# Enum values for task types
int32 CHARGING_TASK = 0
int32 HARVESTING_TASK = 1
int32 UNLOADING_TASK = 2

int32 task_id               # Unique identifier for the task
int32 task_type             # Type/category of the task (choose from constants above)
int32 assigned_robot_id     # Identifier of the robot assigned to this task
int32 target_node_id        # Identifier of the target node for task execution
string job_schedule         # Scheduling information (time/date) for the task
string description          # Brief description of the task
string crop_type            # Type of crop associated with the task (if applicable)
float32 crop_load           # Current load in the basket for the crop

SubTask[] sub_tasks         # List of sub-tasks that make up this task
