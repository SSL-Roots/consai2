
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

class PkAttacker(object):
    def __init__(self, rostime_now):
        self._start_time = rostime_now
        pass

    def get_control_target(self, my_robot_info, ball_info,
        field_length, field_width,
        kazasu_left, kazasu_right, foot_switch_has_pressed, rostime_now):
        control_target = ControlTarget()
        # control_target.goal_velocity = Pose2D(0, 0, 3.14)
        control_target.path.append(Pose2D(-1, -1, 0))

        return control_target
