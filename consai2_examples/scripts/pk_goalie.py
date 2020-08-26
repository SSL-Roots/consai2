
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

class PkGoalie(object):
    def __init__(self):
        pass

    def get_control_target(self, my_robot_info, ball_info,
        field_length, field_width, goal_width,
        kazasu_left, kazasu_right):
        control_target = ControlTarget()
        # control_target.goal_velocity = Pose2D(0, 0, 3.14)
        control_target.path.append(Pose2D(-1, 1, 0))

        return control_target
