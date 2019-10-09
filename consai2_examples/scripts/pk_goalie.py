#!/usr/bin/env python2
# coding: UTF-8

import os
import sys
import copy
import math

import rospy
from consai2_msgs.msg import ControlTarget, RobotCommands, RobotCommand
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo, RobotInfo

import joystick_example

from os.path import expanduser

# ボール情報
ball_info = BallInfo()
# ロボット情報
robot_info = RobotInfo()
# ゴール情報
our_goal_pose = Pose2D()
our_goal_upper = 0
our_goal_lower = 0

# ゴール前までの距離[m]
MARGIN_DIST_X = 0.1

# ボタン操作で目標に生成する移動距離
MOVE_DISTANCE = 0.1

# ゴーリーのコントロール
def make_path(control_target, joy_wrapper, button_lb, button_rb):
    # 制御目標値を生成
    global robot_info, our_goal_pose, our_goal_upper, our_goal_lower
    global MARGIN_DIST_X, MOVE_DISTANCE

    robot_pose = robot_info.pose

    # Trueで走行開始
    control_target.control_enable = True

    # ゴーリーの位置を生成
    target_pose = Pose2D(our_goal_pose.x, robot_pose.y, 0)
    target_pose.x += MARGIN_DIST_X
    if button_lb:
        target_pose.y += MOVE_DISTANCE
    if button_rb:
        target_pose.y -= MOVE_DISTANCE

    # 行き過ぎを抑制する
    if our_goal_upper < target_pose.y:
        target_pose.y = our_goal_upper
    elif target_pose.y < our_goal_lower:
        target_pose.y = our_goal_lower
        
    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

# 特定のボタンを押すとゴール中心へ移動
def move_goal_center(control_target):

    global our_goal_pose

    # ゴール中心から一定距離前に出る
    target_pose = our_goal_pose
    target_pose.x += MOVE_DISTANCE

    control_target.control_enable = True
    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

# コントローラを離しているときは停止
def stop(control_target):
    global robot_info

    # 現在の自分の位置を代入
    target_pose = robot_info.pose

    control_target.control_enable = True
    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

# ボールの情報を取得
def get_ball_info(msg):
    global ball_info
    ball_info = msg

# ロボットの情報を取得
def get_robot_info(msg):
    global robot_info
    robot_info = msg

# フィールドの情報取得
def get_field_info(data):
    global our_goal_pose, our_goal_upper, our_goal_lower
    
    # フィールド長
    field_length = data.field_length

    # ゴール幅
    our_goal_upper = data.goal_width/2
    our_goal_lower = -data.goal_width/2

    # ゴール座標
    our_goal_pose.x = -field_length/2
    our_goal_pose.y = 0


def main():
    
    global robot_info, ball_info

    rospy.init_node('control_example')
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    COLOR = "blue" # 'blue' or 'yellow'
    TARGET_ID = 0 # 0 ~ MAX_ID
    SIDE = "left"

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = COLOR + SIDE + '/consai2_game/control_target_' + COLOR +'_' + topic_id
    topic_name_robot_info = COLOR + SIDE + '/vision_wrapper/robot_info_' + COLOR + '_' + str(TARGET_ID)
    topic_vision = COLOR + SIDE + '/vision_receiver/raw_vision_geometry'

     # print sub
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)
    pub_joy = rospy.Publisher('consai2_examples/joy_target', ControlTarget, queue_size=1)

    # Robotの位置を取得する
    sub_robot = rospy.Subscriber(topic_name_robot_info, RobotInfo, get_robot_info)

    # フィールドの情報をもらう
    sub_vision = rospy.Subscriber(topic_vision, VisionGeometry, get_field_info)

    # ボールの位置を取得
    sub_ball_info = rospy.Subscriber(
            'vision_wrapper/ball_info', BallInfo,
            get_ball_info, queue_size=1)

    # joystick
    joy_wrapper = joystick_example.JoyWrapper()

    # CotrolTargetの生成
    control_target = ControlTarget()
    # ロボットIDを設定
    control_target.robot_id = TARGET_ID

    # 制御目標値を生成
    r = rospy.Rate(60)
    while 1:
        # ボタン情報の取得
        if joy_wrapper.get_button_status() != None:
            _joy_msg = joy_wrapper.get_button_status()
            button_lb = _joy_msg.buttons[4]
            button_rb = _joy_msg.buttons[5]
            button_x  = _joy_msg.buttons[0]
        else:
            button_lb = 0
            button_rb = 0
            button_x  = 0

        # LB or RBボタンでゴール前を左右に移動
        if button_lb or button_rb:
            # パスの生成
            control_target = make_path(control_target, joy_wrapper, button_lb, button_rb)

        # Xボタンを押すとゴール前の真ん中へ移動
        elif button_x:
            control_target = move_goal_center(control_target)

        # 何も押していないときは止まる
        else:
            control_target = stop(control_target)

        pub.publish(control_target)

        r.sleep()

if __name__ == '__main__':
    main()
