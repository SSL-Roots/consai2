
from sensor_msgs.msg import Joy

class JoyWrapper(object):
    def __init__(self):
        pass

    def update(self, joy_msg):
        pass

    def get_attacker_id(self):
        return 1

    def get_attacker_is_yellow(self):
        return False

    def get_goalie_id(self):
        return 0
    
    def get_goalie_is_yellow(self):
        return False

    def get_kazasu_left(self):
        return 0.0

    def get_kazasu_right(self):
        return 0.0

    def get_foot_switch_has_pressed(self):
        return False

    def get_attacker_can_move(self):
        return True

    def get_goalie_can_move(self):
        return True

    def get_attacker_can_use_kazasu_foot(self):
        return False

    def get_goalie_can_use_kazasu_foot(self):
        return False
