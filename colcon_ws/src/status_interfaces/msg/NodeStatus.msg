# header.stamp specifies the ROS time for this measurement 
std_msgs/Header header

# Number of Topics
int32 total
int32 active
int32 inactive

string[] missing_nodes
string[] inactive_nodes
NodeDetails[] active_nodes
