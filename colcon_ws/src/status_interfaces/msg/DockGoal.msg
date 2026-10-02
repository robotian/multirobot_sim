# DockGoal.msg
# Taken from DockRobot.action interface to use as message in our implementations

bool use_dock_id True  # Whether to use the dock_id or dock_pose fields
string dock_id  # Dock name or ID to dock at, from given dock database

geometry_msgs/PoseStamped dock_pose  # Dock pose
string dock_type  # If using dock_pose, what type of dock it is. Not necessary if only using one type of dock.

float32 max_staging_time 1000.0  # Maximum time for navigation to get to the dock's staging pose.
bool navigate_to_staging_pose True  # Whether or not to navigate to staging pose or assume robot is already at staging pose within tolerance to execute behavior
