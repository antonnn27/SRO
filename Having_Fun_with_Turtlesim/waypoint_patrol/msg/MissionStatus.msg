# waypoint_controller -> mission_manager
int32 goal_id      # goal yang sedang dikerjakan (-1 = tidak ada)
float32 x          # pose turtle1 saat ini
float32 y
float32 distance_to_goal
bool reached       # true jika jarak < toleransi
