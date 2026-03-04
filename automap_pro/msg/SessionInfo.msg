# 会话信息消息

uint64 session_id
string session_name
float64 start_time
float64 end_time
int32 num_submaps
int32 num_keyframes
string data_path

# 起始位姿和结束位姿
float64[6] start_pose  # x, y, z, roll, pitch, yaw
float64[6] end_pose    # x, y, z, roll, pitch, yaw

# 统计信息
float64 total_distance

# 状态
bool is_active
bool is_optimized

# 元数据
string sensor_config
string environment_type
float64 map_size

int32[] submap_ids
