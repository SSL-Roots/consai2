#!/usr/bin/env python2
# coding: UTF-8

import rospy
from sensor_msgs.msg import Joy
from consai2_msgs.msg import RobotInfo
from pk_joy_wrapper import JoyWrapper

joy_msg_ = Joy()
def callback_joy(msg):
    global joy_msg_
    joy_msg_ = msg

blue_robot_info_ = {}
def callback_blue_robot_info(msg, robot_id):
    global blue_robot_info_
    blue_robot_info_[robot_id] = msg

yellow_robot_info_ = {}
def callback_yellow_robot_info(msg, robot_id):
    global yellow_robot_info_
    yellow_robot_info_[robot_id] = msg

def get_robot_info(robot_id, is_yellow):
    global blue_robot_info_
    global yellow_robot_info_

    robot_info = False
    if is_yellow:
        if robot_id in yellow_robot_info_.keys():
            return yellow_robot_info_[robot_id]
    else:
        if robot_id in blue_robot_info_.keys():
            return blue_robot_info_[robot_id]

    return False


def main():
    global joy_msg_
    global blue_robot_info_
    global yellow_robot_info_

    MAX_ID = rospy.get_param('consai2_description/max_id')

    rospy.init_node('pk_example')

    joy_wrapper = JoyWrapper()

    sub_joy = rospy.Subscriber('joy', Joy, callback_joy, queue_size=1)
    sub_robot_info_list = []
    for robot_id in range(MAX_ID):
        # 末尾に16進数の文字列をつける
        topic_id = hex(robot_id)[2:]

        topic_name = 'vision_wrapper/robot_info_blue_' + topic_id
        sub_robot_info = rospy.Subscriber(topic_name, RobotInfo,
            callback_blue_robot_info, queue_size=1, callback_args=robot_id)
        sub_robot_info_list.append(sub_robot_info)

        topic_name = 'vision_wrapper/robot_info_yellow_' + topic_id
        sub_robot_info = rospy.Subscriber(topic_name, RobotInfo,
            callback_yellow_robot_info, queue_size=1, callback_args=robot_id)
        sub_robot_info_list.append(sub_robot_info)

    r = rospy.Rate(60)
    rospy.loginfo("pk_example start!")
    while not rospy.is_shutdown():
        # rospy.loginfo(joy_msg_)

        joy_wrapper.update(joy_msg_)

        goalie_id = joy_wrapper.get_goalie_id()
        attacker_id = joy_wrapper.get_attacker_id()
        goalie_is_yellow = joy_wrapper.get_goalie_is_yellow()
        attacker_is_yellow = joy_wrapper.get_attacker_is_yellow()

        goalie_info = get_robot_info(goalie_id, goalie_is_yellow)
        attacker_info = get_robot_info(attacker_id, attacker_is_yellow)

        if goalie_info is False:
            rospy.logwarn("No goalie robot info.")
            continue

        if attacker_info is False:
            rospy.logwarn("No attacker robot info.")
            continue



        r.sleep()

if __name__ == '__main__':
    main()