std_msgs/Header header

# Detection metadata
string detector_id        # e.g. "lavender", "weed", "obstacle"
float32 score
bool detection_valid      # True if at least one usable 3D point exists

# Bounding box in pixels — always populated when score >= threshold
int32 bbox_x1
int32 bbox_y1
int32 bbox_x2
int32 bbox_y2

# 3D center in camera frame — valid flag guards usage
bool center_valid
geometry_msgs/Point center

# 4 corners in camera frame — each independently valid
bool corner_tl_valid
geometry_msgs/Point corner_tl

bool corner_tr_valid
geometry_msgs/Point corner_tr

bool corner_bl_valid
geometry_msgs/Point corner_bl

bool corner_br_valid
geometry_msgs/Point corner_br

# Physical extents of the bounding box in metres
bool extents_valid
geometry_msgs/Vector3 extents