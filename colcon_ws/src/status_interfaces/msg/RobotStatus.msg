# Time (timestamp with time zone)
std_msgs/Header header

# Robot Name (string)
string robot_namespace

# Online Flag (string)
int32 online_flag

# Status (string)
int32 status

# GPS Location (point with x, y, z coordinates)
geometry_msgs/Point gps_location

# Task (string)
string task

# Topo Map Position (XYZ)
geometry_msgs/Point topo_map_position

# Topo Map Orientation (RPY)
geometry_msgs/Quaternion topo_map_orientation

# Current Node ID (integer)
int32 current_node_id

# Target Node ID (integer)
int32 target_node_id

# Battery Level (real number)
float32 battery_level

# Operation Hours After Charging (real number)
string operation_hours_after_charging

# Load Status (real number)
float32 load_status

# Crop Type (string)
string crop_type