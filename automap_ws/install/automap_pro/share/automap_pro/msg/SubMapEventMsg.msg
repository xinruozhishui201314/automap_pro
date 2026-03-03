std_msgs/Header header
int32  submap_id
uint64 session_id
string event_type           # "CREATED" | "FROZEN" | "OPTIMIZED" | "ARCHIVED"
int32  keyframe_count
float64 spatial_extent_m
geometry_msgs/Pose anchor_pose
bool has_valid_gps
