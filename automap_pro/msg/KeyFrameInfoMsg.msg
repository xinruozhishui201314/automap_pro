# Extended keyframe info from FAST-LIVO2 composable node
std_msgs/Header header
uint64 keyframe_id
float64 timestamp

# ESIKF state quality
float64 esikf_covariance_norm    # Frobenius norm of covariance matrix
bool    is_degenerate            # Whether ESIKF is degenerate
float32 map_update_rate          # Points updated per second

# IMU bias
float64[3] gyro_bias
float64[3] accel_bias

# Point cloud validity
uint32 cloud_point_count
bool   cloud_valid

# Pose with full covariance
geometry_msgs/PoseWithCovariance pose
