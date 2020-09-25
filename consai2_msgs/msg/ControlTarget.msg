# control target for consai2_game and consai2_control

uint8   robot_id

bool control_enable
# path[last] == goal_pose
geometry_msgs/Pose2D[] path
geometry_msgs/Pose2D goal_velocity
geometry_msgs/Pose2D[] max_velocity
geometry_msgs/Pose2D[] max_acceleration

# 0.0 ~ 1.0
float64 kick_power
# True/False
bool    chip_enable
# 0.0 ~ 1.0
float64 dribble_power

# default:unused
bool    test_flag
# default:unused
float64 test_param

