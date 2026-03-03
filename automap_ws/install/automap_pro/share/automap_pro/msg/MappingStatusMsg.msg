std_msgs/Header header
string state   # "IDLE" | "MAPPING" | "OPTIMIZING" | "SAVING"
uint64 session_id
uint32 keyframe_count
uint32 submap_count
uint32 loop_count
bool   gps_aligned
float32 gps_alignment_score
float64 map_entropy
float64 total_distance_m
