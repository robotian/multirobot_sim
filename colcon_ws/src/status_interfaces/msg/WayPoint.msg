# WayPoint.msg
# Represents a waypoint in 2D/3D space, optionally linked to a node.

int32 node_id      # Identifier of the node associated with this waypoint
float64 x          # X-coordinate in the map or world frame
float64 y          # Y-coordinate in the map or world frame
float64 theta      # Heading angle of the robot in the map or world frame
