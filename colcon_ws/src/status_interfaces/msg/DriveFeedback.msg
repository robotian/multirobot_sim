# status values — mirrors husky_operations_manager's DriveStatus enum
int32 IDLE=0
int32 SCANNING=1
int32 CONTROLLING=2
int32 STOPPED=3
int32 DEPARTING=4
int32 CANCELED=5
int32 ERROR=6
int32 ABORTED=7

std_msgs/Header header
int32 status
float32 target_x
float32 target_y
float32 target_yaw
float32 current_x
float32 current_y
float32 current_yaw
float32 distance_error
float32 heading_error
float32 cmd_linear_x
float32 cmd_angular_z
int32 harvested_count
float32 state_timer
