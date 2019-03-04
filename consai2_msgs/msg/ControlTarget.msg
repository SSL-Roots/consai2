# control target for consai2_game and consai2_control

uint8   robot_id

bool control_enable
geometry_msgs/Pose2D goal_pose
geometry_msgs/Pose2D goal_velocity

bool use_path
# path[last] == goal_pose
geometry_msgs/Pose2D[] path

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

