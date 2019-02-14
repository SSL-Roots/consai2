#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
from consai2_msgs.msg import RobotCommand, RobotCommands
from sensor_msgs.msg import Joy


class JoyWrapper(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber('joy', Joy, self._callback_joy, queue_size=1)
        self._pub_commands = rospy.Publisher('consai2_control/robot_commands', RobotCommands, queue_size=1)

        # /consai2_examples/launch/joystick_example.launch でキー割り当てを変更する
        self._BUTTON_SHUTDOWN_1 = rospy.get_param('~button_shutdown_1')
        self._BUTTON_SHUTDOWN_2 = rospy.get_param('~button_shutdown_2')

        self._BUTTON_MOVE_ENABLE = rospy.get_param('~button_move_enable')
        self._AXIS_VEL_SURGE   = rospy.get_param('~axis_vel_surge')
        self._AXIS_VEL_SWAY   = rospy.get_param('~axis_vel_sway')
        self._AXIS_VEL_ANGULAR   = rospy.get_param('~axis_vel_angular')

        self._BUTTON_KICK_ENABLE = rospy.get_param('~button_kick_enable')
        self._BUTTON_KICK_STRAIGHT = rospy.get_param('~button_kick_straight')
        self._BUTTON_KICK_CHIP = rospy.get_param('~button_kick_chip')

        self._BUTTON_DRIBBLE_ENABLE = rospy.get_param('~button_dribble_enable')


        self._MAX_VEL_SURGE = 1.0
        self._MAX_VEL_SWAY = 1.0
        self._MAX_VEL_ANGULAR = math.pi
        self._MAX_KICK_POWER = 1.0 # DO NOT EDIT
        self._MAX_DRIBBLE_POWER = 1.0 # DO NOT EDIT


    def _callback_joy(self, msg):
        robot_commands = RobotCommands()
        robot_commands.is_yellow = False
        robot_commands.header.stamp = rospy.Time.now()

        command = RobotCommand()
        command.robot_id = 0

        # シャットダウン
        if msg.buttons[self._BUTTON_SHUTDOWN_1] and msg.buttons[self._BUTTON_SHUTDOWN_2]:
            rospy.signal_shutdown('finish')
            return

        # 走行
        if msg.buttons[self._BUTTON_MOVE_ENABLE]:
            command.vel_surge = msg.axes[self._AXIS_VEL_SURGE] * self._MAX_VEL_SURGE
            command.vel_sway = msg.axes[self._AXIS_VEL_SWAY] * self._MAX_VEL_SWAY
            command.vel_angular = msg.axes[self._AXIS_VEL_ANGULAR] * self._MAX_VEL_ANGULAR

        # キック
        if msg.buttons[self._BUTTON_KICK_ENABLE]:
            if msg.buttons[self._BUTTON_KICK_STRAIGHT]:
                command.kick_power = self._MAX_KICK_POWER
            elif msg.buttons[self._BUTTON_KICK_CHIP]:
                command.kick_power = self._MAX_KICK_POWER
                command.chip_enable = True

        # ドリブラー
        if msg.buttons[self._BUTTON_DRIBBLE_ENABLE]:
            command.dribble_power = self._MAX_DRIBBLE_POWER

        robot_commands.commands.append(command)

        self._pub_commands.publish(robot_commands)


def main():
    rospy.init_node('joystick_sim')

    joy_wrapper = JoyWrapper()

    rospy.spin()

if __name__ == '__main__':
    main()
