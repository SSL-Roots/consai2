# coding: UTF-8

import math
import rospy
from sensor_msgs.msg import Joy

class JoyWrapper(object):
    def __init__(self):
        self._MAX_ID = rospy.get_param('consai2_description/max_id', 10)

        self._BUTTON_FOOT_SWITCH = rospy.get_param('~button_foot_switch')
        self._AXIS_KAZASU_LEFT_X = rospy.get_param('~axis_kazasu_left_x')
        self._AXIS_KAZASU_LEFT_Y = rospy.get_param('~axis_kazasu_left_y')
        self._AXIS_KAZASU_RIGHT_X = rospy.get_param('~axis_kazasu_right_x')
        self._AXIS_KAZASU_RIGHT_Y = rospy.get_param('~axis_kazasu_right_y')

        self._BUTTON_ATTACKER_CONFIG_ENABLE = rospy.get_param('~button_attacker_config_enable')
        self._BUTTON_GOALIE_CONFIG_ENABLE = rospy.get_param('~button_goalie_config_enable')
        self._BUTTON_ATTACKER_CAN_MOVE = rospy.get_param('~button_attacker_can_move')
        self._BUTTON_GOALIE_CAN_MOVE = rospy.get_param('~button_goalie_can_move')
        self._BUTTON_ATTACKER_KAZAFOO_AVAILABLE = rospy.get_param('~button_attacker_kazafoo_available')
        self._BUTTON_GOALIE_KAZAFOO_AVAILABLE = rospy.get_param('~button_goalie_kazafoo_available')

        self._ANALOG_D_PAD = rospy.get_param('~analog_d_pad')
        self._D_PAD_UP = rospy.get_param('~d_pad_up')
        self._D_PAD_DOWN = rospy.get_param('~d_pad_down')
        self._D_PAD_LEFT = rospy.get_param('~d_pad_left')
        self._D_PAD_RIGHT = rospy.get_param('~d_pad_right')
        self._D_PAD_UP_IS_POSITIVE = rospy.get_param('~d_pad_up_is_positive')
        self._D_PAD_RIGHT_IS_POSITIVE = rospy.get_param('~d_pad_right_is_positive')

        self._D_PAD_ID_INCREMENT = rospy.get_param('~d_pad_id_increment')
        self._D_PAD_ID_DECREMENT = rospy.get_param('~d_pad_id_decrement')
        self._D_PAD_COLOR_TOGGLE = rospy.get_param('~d_pad_color_toggle')

        self._foot_switch_has_pressed = False
        self._kazasu_left = False
        self._kazasu_right = False
        self._attacker_can_move = False
        self._goalie_can_move = False
        self._attacker_id = 0
        self._goalie_id = 0
        self._attacker_is_yellow = False
        self._goalie_is_yellow = True
        self._attacker_kazafoo_is_available = False
        self._goalie_kazafoo_is_available = False

        self._attacker_move_state_is_changing = False
        self._attacker_id_is_changing = False
        self._attacker_color_is_changing = False
        self._goalie_move_state_is_changing = False
        self._goalie_id_is_changing = False
        self._goalie_color_is_changing = False
        self._attacker_kazafoo_availability_is_changing = False
        self._goalie_kazafoo_availability_is_changing = False

    def update(self, joy_msg):
        if not joy_msg.buttons or not joy_msg.axes:
            rospy.logwarn("ジョイコンのメッセージが来てないよ")
            return

        if joy_msg.buttons[self._BUTTON_FOOT_SWITCH]:
            self._foot_switch_has_pressed = True
        else:
            self._foot_switch_has_pressed = False

        # アナログスティックをどこに傾けても、0.0 ~ 1.0を得る
        kazasu_analog_left = max(
            math.fabs(joy_msg.axes[self._AXIS_KAZASU_LEFT_X]),
            math.fabs(joy_msg.axes[self._AXIS_KAZASU_LEFT_Y]),
            )
        kazasu_analog_right = max(
            math.fabs(joy_msg.axes[self._AXIS_KAZASU_RIGHT_X]),
            math.fabs(joy_msg.axes[self._AXIS_KAZASU_RIGHT_Y]),
            )

        if kazasu_analog_left > 0.5:
            self._kazasu_left = True
        else:
            self._kazasu_left = False

        if kazasu_analog_right > 0.5:
            self._kazasu_right = True
        else:
            self._kazasu_right = False

        state_has_changed = False
        state_has_changed |= self._change_attacker_move_state(joy_msg)
        state_has_changed |= self._change_goalie_move_state(joy_msg)
        state_has_changed |= self._change_attacker_id(joy_msg)
        state_has_changed |= self._change_goalie_id(joy_msg)
        state_has_changed |= self._toggle_attacker_color(joy_msg)
        state_has_changed |= self._toggle_goalie_color(joy_msg)
        state_has_changed |= self._change_attacker_kazafoo_availability(joy_msg)
        state_has_changed |= self._change_goalie_kazafoo_availability(joy_msg)

        # デバッグ用のメッセージ（本番運用でもメッセージを出力したほうが作業しやすい）
        if state_has_changed:
            text =  "attacker(Yellow:" + str(self._attacker_is_yellow) \
                + "\tID:" + str(self._attacker_id) \
                + "\tMove:" + str(self._attacker_can_move) \
                + "\tKazafoo:" + str(self._attacker_kazafoo_is_available) \
                + ")attacker\n" \
                + "goalie(Yellow:" + str(self._goalie_is_yellow) \
                + "\tID:" + str(self._goalie_id) \
                + "\tMove:" + str(self._goalie_can_move) \
                + "\tKazafoo:" + str(self._goalie_kazafoo_is_available) \
                + ")goalie"
            print(text)

    def get_attacker_id(self):
        return self._attacker_id

    def get_attacker_is_yellow(self):
        return self._attacker_is_yellow

    def get_goalie_id(self):
        return self._goalie_id
    
    def get_goalie_is_yellow(self):
        return self._goalie_is_yellow

    def get_kazasu_left(self):
        return self._kazasu_left

    def get_kazasu_right(self):
        return self._kazasu_right

    def get_foot_switch_has_pressed(self):
        return self._foot_switch_has_pressed

    def get_attacker_can_move(self):
        return self._attacker_can_move

    def get_goalie_can_move(self):
        return self._goalie_can_move

    def get_attacker_can_use_kazasu_foot(self):
        return self._attacker_kazafoo_is_available

    def get_goalie_can_use_kazasu_foot(self):
        return self._goalie_kazafoo_is_available

    def _change_attacker_move_state(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_ATTACKER_CAN_MOVE]:
            if joy_msg.buttons[self._BUTTON_ATTACKER_CONFIG_ENABLE]:
                if not self._attacker_move_state_is_changing:
                    self._attacker_can_move = not self._attacker_can_move
                    self._attacker_move_state_is_changing = True
                    state_has_changed = True
            else:
                # Enableが押されていないときはロボットを停止する
                self._attacker_can_move = False
                state_has_changed = True
        else:
            self._attacker_move_state_is_changing = False

        return state_has_changed

    def _change_goalie_move_state(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_GOALIE_CAN_MOVE]:
            if joy_msg.buttons[self._BUTTON_GOALIE_CONFIG_ENABLE]:
                if not self._goalie_move_state_is_changing:
                    self._goalie_can_move = not self._goalie_can_move
                    self._goalie_move_state_is_changing = True
                    state_has_changed = True
            else:
                # Enableが押されていないときはロボットを停止する
                self._goalie_can_move = False
                state_has_changed = True
        else:
            self._goalie_move_state_is_changing = False
        
        return state_has_changed

    def _change_attacker_id(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_ATTACKER_CONFIG_ENABLE]:
            if self._d_pad(joy_msg, self._D_PAD_ID_INCREMENT):
                if self._attacker_id < self._MAX_ID \
                    and not self._attacker_id_is_changing:
                    self._attacker_id += 1
                    self._attacker_id_is_changing = True
                    state_has_changed = True

            elif self._d_pad(joy_msg, self._D_PAD_ID_DECREMENT):
                if self._attacker_id > 0 \
                    and not self._attacker_id_is_changing:
                    self._attacker_id -= 1
                    self._attacker_id_is_changing = True
                    state_has_changed = True

            else:
                self._attacker_id_is_changing = False

        return state_has_changed

    def _change_goalie_id(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_GOALIE_CONFIG_ENABLE]:
            if self._d_pad(joy_msg, self._D_PAD_ID_INCREMENT):
                if self._goalie_id < self._MAX_ID \
                    and not self._goalie_id_is_changing:
                    self._goalie_id += 1
                    self._goalie_id_is_changing = True
                    state_has_changed = True

            elif self._d_pad(joy_msg, self._D_PAD_ID_DECREMENT):
                if self._goalie_id > 0 \
                    and not self._goalie_id_is_changing:
                    self._goalie_id -= 1
                    self._goalie_id_is_changing = True
                    state_has_changed = True

            else:
                self._goalie_id_is_changing = False

        return state_has_changed

    def _toggle_attacker_color(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_ATTACKER_CONFIG_ENABLE]:
            if self._d_pad(joy_msg, self._D_PAD_COLOR_TOGGLE):
                if not self._attacker_color_is_changing:
                    self._attacker_is_yellow = not self._attacker_is_yellow
                    self._attacker_color_is_changing = True
                    state_has_changed = True
            else:
                self._attacker_color_is_changing = False
        
        return state_has_changed

    def _toggle_goalie_color(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_GOALIE_CONFIG_ENABLE]:
            if self._d_pad(joy_msg, self._D_PAD_COLOR_TOGGLE):
                if not self._goalie_color_is_changing:
                    self._goalie_is_yellow = not self._goalie_is_yellow
                    self._goalie_color_is_changing = True
                    state_has_changed = True
            else:
                self._goalie_color_is_changing = False

        return state_has_changed

    def _change_attacker_kazafoo_availability(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_ATTACKER_KAZAFOO_AVAILABLE]:
            if joy_msg.buttons[self._BUTTON_ATTACKER_CONFIG_ENABLE]:
                if not self._attacker_kazafoo_availability_is_changing:
                    self._attacker_kazafoo_is_available = not self._attacker_kazafoo_is_available
                    self._attacker_kazafoo_availability_is_changing = True
                    state_has_changed = True
            else:
                self._attacker_kazafoo_is_available = False
                state_has_changed = True
        else:
            self._attacker_kazafoo_availability_is_changing = False

        return state_has_changed

    def _change_goalie_kazafoo_availability(self, joy_msg):
        state_has_changed = False
        if joy_msg.buttons[self._BUTTON_GOALIE_KAZAFOO_AVAILABLE]:
            if joy_msg.buttons[self._BUTTON_GOALIE_CONFIG_ENABLE]:
                if not self._goalie_kazafoo_availability_is_changing:
                    self._goalie_kazafoo_is_available = not self._goalie_kazafoo_is_available
                    self._goalie_kazafoo_availability_is_changing = True
                    state_has_changed = True
            else:
                self._goalie_kazafoo_is_available = False
                state_has_changed = True
        else:
            self._goalie_kazafoo_availability_is_changing = False
        
        return state_has_changed

    def _joy_d_pad(self, joy_msg, target_pad, positive_on):
        if self._ANALOG_D_PAD:
            if positive_on:
                return joy_msg.axes[target_pad] > 0
            else:
                return joy_msg.axes[target_pad] < 0
        else:
            return joy_msg.buttons[target_pad]

    def _d_pad_up(self, joy_msg):
        positive_on = self._D_PAD_UP_IS_POSITIVE
        return self._joy_d_pad(joy_msg, self._D_PAD_UP, positive_on)

    def _d_pad_down(self, joy_msg):
        positive_on = not self._D_PAD_UP_IS_POSITIVE
        return self._joy_d_pad(joy_msg, self._D_PAD_DOWN, positive_on)

    def _d_pad_left(self, joy_msg):
        positive_on = not self._D_PAD_RIGHT_IS_POSITIVE
        return self._joy_d_pad(joy_msg, self._D_PAD_LEFT, positive_on)

    def _d_pad_right(self, joy_msg):
        positive_on = self._D_PAD_RIGHT_IS_POSITIVE
        return self._joy_d_pad(joy_msg, self._D_PAD_RIGHT, positive_on)

    def _d_pad(self, joy_msg, target):
        if target == "up":
            return self._d_pad_up(joy_msg)
        elif target == "down":
            return self._d_pad_down(joy_msg)
        elif target == "left":
            return self._d_pad_left(joy_msg)
        elif target == "right":
            return self._d_pad_right(joy_msg)
        else:
            return False
