#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
import copy
from consai2_msgs.msg import RobotCommand, RobotCommands
from sensor_msgs.msg import Joy


class JoyWrapper(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber('joy', Joy, self._callback_joy, queue_size=1)
        self._pub_commands = rospy.Publisher('consai2_control/robot_commands', RobotCommands, queue_size=1)

        # /consai2_examples/launch/joystick_example.launch でキー割り当てを変更する
        self._BUTTON_SHUTDOWN_1     = rospy.get_param('~button_shutdown_1')
        self._BUTTON_SHUTDOWN_2     = rospy.get_param('~button_shutdown_2')

        self._BUTTON_MOVE_ENABLE    = rospy.get_param('~button_move_enable')
        self._AXIS_VEL_SURGE        = rospy.get_param('~axis_vel_surge')
        self._AXIS_VEL_SWAY         = rospy.get_param('~axis_vel_sway')
        self._AXIS_VEL_ANGULAR      = rospy.get_param('~axis_vel_angular')

        self._BUTTON_KICK_ENABLE    = rospy.get_param('~button_kick_enable')
        self._BUTTON_KICK_STRAIGHT  = rospy.get_param('~button_kick_straight')
        self._BUTTON_KICK_CHIP      = rospy.get_param('~button_kick_chip')
        self._AXIS_KICK_POWER       = rospy.get_param('~axis_kick_power')

        self._BUTTON_DRIBBLE_ENABLE = rospy.get_param('~button_dribble_enable')
        self._AXIS_DRIBBLE_POWER    = rospy.get_param('~axis_dribble_power')

        self._BUTTON_ID_ENABLE      = rospy.get_param('~button_id_enable')
        self._AXIS_ID_CHANGE        = rospy.get_param('~axis_id_change')

        self._BUTTON_COLOR_ENABLE   = rospy.get_param('~button_color_enable')
        self._AXIS_COLOR_CHANGE     = rospy.get_param('~axis_color_change')

        self._BUTTON_ALL_ID_1       = rospy.get_param('~button_all_id_1')
        self._BUTTON_ALL_ID_2       = rospy.get_param('~button_all_id_2')
        self._BUTTON_ALL_ID_3       = rospy.get_param('~button_all_id_3')
        self._BUTTON_ALL_ID_4       = rospy.get_param('~button_all_id_4')


        #TODO:MAX_IDを設定するファイルを決める
        self._MAX_ID = rospy.get_param('max_id')

        self._MAX_VEL_SURGE = 1.0
        self._MAX_VEL_SWAY = 1.0
        self._MAX_VEL_ANGULAR = math.pi

        self._MAX_KICK_POWER = 1.0 # DO NOT EDIT
        self._KICK_POWER_CONTROL = 0.1

        self._MAX_DRIBBLE_POWER = 1.0 # DO NOT EDIT
        self._DRIBBLE_POWER_CONTROL = 0.1

        self._kick_power = 0.5
        self._dribble_power = 0.5
        self._robot_id = 0
        self._is_yellow = False
        self._all_member = False


    def _callback_joy(self, msg):
        robot_commands = RobotCommands()
        robot_commands.header.stamp = rospy.Time.now()

        command = RobotCommand()

        # シャットダウン
        if msg.buttons[self._BUTTON_SHUTDOWN_1] and msg.buttons[self._BUTTON_SHUTDOWN_2]:
            rospy.signal_shutdown('finish')
            return

        # チームカラーの変更
        if msg.buttons[self._BUTTON_COLOR_ENABLE]:
            if math.fabs(msg.axes[self._AXIS_COLOR_CHANGE]) > 0:
                self._is_yellow = not self._is_yellow
            print 'is_yellow: ' + str(self._is_yellow)

        # IDの変更
        if msg.buttons[self._BUTTON_ID_ENABLE]:
            self._robot_id += int(msg.axes[self._AXIS_ID_CHANGE])

            if self._robot_id > self._MAX_ID:
                self._robot_id = self._MAX_ID
            if self._robot_id < 0:
                self._robot_id = 0
            print 'robot_id:' + str(self._robot_id)

        # 全ID操作の変更
        if msg.buttons[self._BUTTON_ALL_ID_1] and \
                msg.buttons[self._BUTTON_ALL_ID_2] and \
                msg.buttons[self._BUTTON_ALL_ID_3] and \
                msg.buttons[self._BUTTON_ALL_ID_4]:
            self._all_member = not self._all_member
            print 'all_member: ' + str(self._all_member)

        # 走行
        if msg.buttons[self._BUTTON_MOVE_ENABLE]:
            command.vel_surge = msg.axes[self._AXIS_VEL_SURGE] * self._MAX_VEL_SURGE
            command.vel_sway = msg.axes[self._AXIS_VEL_SWAY] * self._MAX_VEL_SWAY
            command.vel_angular = msg.axes[self._AXIS_VEL_ANGULAR] * self._MAX_VEL_ANGULAR

        # キック
        if msg.buttons[self._BUTTON_KICK_ENABLE]:
            if msg.buttons[self._BUTTON_KICK_STRAIGHT]:
                command.kick_power = self._kick_power
            elif msg.buttons[self._BUTTON_KICK_CHIP]:
                command.kick_power = self._kick_power
                command.chip_enable = True

            # パワーの変更
            axis_value = msg.axes[self._AXIS_KICK_POWER]
            if math.fabs(axis_value) > 0:
                self._kick_power += math.copysign(self._KICK_POWER_CONTROL, axis_value)
                if self._kick_power > self._MAX_KICK_POWER:
                    self._kick_power = self._MAX_KICK_POWER
                if self._kick_power < 0.001:
                    self._kick_power = 0.0
                print 'kick_power :' + str(self._kick_power)

        # ドリブラー
        if msg.buttons[self._BUTTON_DRIBBLE_ENABLE]:
            command.dribble_power = self._dribble_power

            # パワーの変更
            axis_value = msg.axes[self._AXIS_DRIBBLE_POWER]
            if math.fabs(axis_value) > 0:
                self._dribble_power += math.copysign(self._DRIBBLE_POWER_CONTROL, axis_value)
                if self._dribble_power > self._MAX_DRIBBLE_POWER:
                    self._dribble_power = self._MAX_DRIBBLE_POWER
                if self._dribble_power < 0.001:
                    self._dribble_power = 0.0
                print 'dribble_power:' + str(self._dribble_power)


        # チームカラーをセット
        robot_commands.is_yellow = self._is_yellow

        # IDとコマンドをセット
        if self._all_member:
            # 全IDのロボットを動かす
            for robot_id in range(self._MAX_ID + 1):
                command.robot_id = robot_id
                robot_commands.commands.append(copy.deepcopy(command))
        else:
            command.robot_id = self._robot_id
            robot_commands.commands.append(command)

        self._pub_commands.publish(robot_commands)


def main():
    rospy.init_node('joystick_example')

    joy_wrapper = JoyWrapper()

    rospy.spin()

if __name__ == '__main__':
    main()
