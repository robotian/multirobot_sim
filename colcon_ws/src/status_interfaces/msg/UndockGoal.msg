# UndockGoal.msg
# Taken from UndockRobot.action interface to use as message in our implementations

string dock_type                   # Dock name or ID to dock at, from given dock database
float32 max_undocking_time 30.0    # Maximum time to undock
