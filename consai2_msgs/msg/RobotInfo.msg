# robot information for consai2_game

uint8                robot_id

# Robot 2D pose
# This field will be deprecate
geometry_msgs/Pose2D pose
geometry_msgs/Pose2D velocity
geometry_msgs/Twist  velocity_twist
bool                 detected

# Latest vision detection time
# [Deprecated] This field will be removed. 
time                 detection_stamp

geometry_msgs/Pose2D last_detection_pose
bool                 disappeared
