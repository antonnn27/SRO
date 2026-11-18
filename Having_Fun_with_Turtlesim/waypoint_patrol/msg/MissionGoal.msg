# mission_manager -> waypoint_controller
int32 goal_id      # naik setiap goal baru (mencegah status basi dianggap tiba)
float32 x
float32 y
bool active        # false = berhenti / tidak ada goal
