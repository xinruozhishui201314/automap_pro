# 跨会话约束消息
# 用于表示不同会话之间的几何约束

uint64 session_id1
int32 submap_id1
uint64 session_id2
int32 submap_id2

# 变换矩阵（session1到session2）
float64[16] transform

# 信息矩阵（6x6上三角）
float64[21] information

# 匹配质量
float64 fitness_score
bool is_verified
float64 timestamp
