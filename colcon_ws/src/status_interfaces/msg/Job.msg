# Time (timestamp with time zone)
std_msgs/Header header

# Unique identifier for the job
int32 id

# Identifier for the node associated with the job
int32 node_id

# Type of crop related to the job
string crop_type

# Indicates if the crop is ready for harvest (e.g., 0 for not ready, 1 for ready)
int32 harvest_ready

# Current progress status of the job (e.g., 0.0 for not started, 1.0 for completed)
float32 progress_status

# Identifier for the robot assigned to the job
int32 assigned_robot_id

# Latitude coordinate for the job location
float32 latitude

# Longitude coordinate for the job location
float32 longitude
