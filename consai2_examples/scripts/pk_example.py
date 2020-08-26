#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import BallInfo
from consai2_msgs.msg import ControlTarget
from consai2_msgs.msg import RobotInfo
from consai2_msgs.msg import VisionGeometry
from geometry_msgs.msg import Pose2D
from pk_joy_wrapper import JoyWrapper
from sensor_msgs.msg import Joy

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

ball_info_ = BallInfo()
def callback_ball_info(msg):
    global ball_info_
    ball_info_ = msg

vision_geometry_ = None
def callback_vision_geometry(msg):
    global vision_geometry_
    vision_geometry_ = msg

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

def get_field_size():
    global vision_geometry_

    field_length = False
    field_width = False
    goal_width = False
    if vision_geometry_:
        field_length = vision_geometry_.field_length
        field_width = vision_geometry_.field_width
        goal_width = vision_geometry_.goal_width

    return field_length, field_width, goal_width

def publish_control_target(blue_publisher_list, yellow_publisher_list, is_yellow,
    robot_id, control_target):
    if is_yellow:
        yellow_publisher_list[robot_id].publish(control_target)
    else:
        blue_publisher_list[robot_id].publish(control_target)


def main():
    global joy_msg_
    global ball_info_

    MAX_ID = rospy.get_param('consai2_description/max_id')

    rospy.init_node('pk_example')

    joy_wrapper = JoyWrapper()

    sub_joy = rospy.Subscriber('joy', Joy, callback_joy, queue_size=1)
    sub_ball_info = rospy.Subscriber('vision_wrapper/ball_info', BallInfo,
        callback_ball_info, queue_size=1)
    sub_vision_geometry = rospy.Subscriber('vision_receiver/raw_vision_geometry',
        VisionGeometry, callback_vision_geometry, queue_size=1)
    sub_robot_info_list = []
    pub_blue_control_target_list = []
    pub_yellow_control_target_list = []
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

        topic_name = 'consai2_game/control_target_blue_' + topic_id
        pub_control_target = rospy.Publisher(topic_name, ControlTarget, queue_size=1)
        pub_blue_control_target_list.append(pub_control_target)

        topic_name = 'consai2_game/control_target_yellow_' + topic_id
        pub_control_target = rospy.Publisher(topic_name, ControlTarget, queue_size=1)
        pub_yellow_control_target_list.append(pub_control_target)

    r = rospy.Rate(60)
    rospy.loginfo("pk_example start!")
    # ID、カラー変更時に制御を止めるため、IDとカラーを保存する
    prev_goalie_is_yellow = False
    prev_attacker_is_yellow = False
    prev_goalie_id = 0
    prev_attacker_id = 0
    start_time_secs = rospy.get_rostime().secs
    while not rospy.is_shutdown():
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

        field_length, field_width, goal_width = get_field_size()

        if not field_length or not field_width or not goal_width:
            rospy.logwarn("No field info.")
            continue

        # 5秒経ったらIDとカラーを変える
        if rospy.get_rostime().secs - start_time_secs > 4:
            attacker_is_yellow = True
            goalie_is_yellow = True
            attacker_id = 4
            goalie_id = 3

        goalie_control_target = ControlTarget()
        goalie_control_target.control_enable = True
        goalie_control_target.robot_id = goalie_id
        goalie_control_target.goal_velocity = Pose2D(0, 0, 3.14)

        attacker_control_target = ControlTarget()
        attacker_control_target.control_enable = True
        attacker_control_target.robot_id = attacker_id
        attacker_control_target.goal_velocity = Pose2D(0, 0, -3.14)

        # 制御目標値の送信
        publish_control_target(
            pub_blue_control_target_list, pub_yellow_control_target_list,
            goalie_is_yellow, goalie_id, goalie_control_target)
        publish_control_target(
            pub_blue_control_target_list, pub_yellow_control_target_list,
            attacker_is_yellow, attacker_id, attacker_control_target)

        # 前回とIDもしくはカラーが変わった場合は、制御停止信号を送信する
        stop_control_target = ControlTarget()
        stop_control_target.control_enable = True
        stop_control_target.goal_velocity = Pose2D()
        if prev_goalie_is_yellow != goalie_is_yellow or prev_goalie_id != goalie_id:
            stop_control_target.robot_id = prev_goalie_id
            publish_control_target(
                pub_blue_control_target_list, pub_yellow_control_target_list,
                prev_goalie_is_yellow, prev_goalie_id, stop_control_target)

        if prev_attacker_is_yellow != attacker_is_yellow or prev_attacker_id != attacker_id:
            stop_control_target.robot_id = prev_attacker_id 
            publish_control_target(
                pub_blue_control_target_list, pub_yellow_control_target_list,
                prev_attacker_is_yellow, prev_attacker_id, stop_control_target)
        
        # IDとカラーを保存する
        prev_goalie_is_yellow = goalie_is_yellow
        prev_attacker_is_yellow = attacker_is_yellow
        prev_goalie_id = goalie_id
        prev_attacker_id = attacker_id

        r.sleep()
    # end while
    
    rospy.loginfo("Finish pk_example.")

if __name__ == '__main__':
    main()