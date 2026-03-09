std_msgs/Header header
int32  submap_i
int32  submap_j
uint64 session_i
uint64 session_j
float32 overlap_score
float32 inlier_ratio
float32 rmse
bool   is_inter_session
float64[36] information_matrix
geometry_msgs/Pose delta_pose
string status   # "ACCEPTED" | "REJECTED"
