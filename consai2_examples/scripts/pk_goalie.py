#!/usr/bin/env python2
# coding: UTF-8

import rospy
import copy

from consai2_msgs.msg import ControlTarget, RobotCommands, RobotCommand
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo, RobotInfo

import joystick_example

import math

robot_pose = Pose2D()
our_goal_pose = Pose2D()
our_goal_upper = 0
our_goal_lower = 0


# ゴーリーのコントロールJ?
def make_path(target_id, joy_wrapper, button_lb, button_rb):
    # 制御目標値を生成
    global robot_pose, our_goal_pose, our_goal_upper, our_goal_lower

    # control_targetの生成
    control_target = ControlTarget()

    # ロボットID
    control_target.robot_id = target_id
    # Trueで走行開始
    control_target.control_enable = True

    # ゴールライン上ではなく一定距離[m]前を守るための変数
    MARGIN_DIST_X = 0.1

    # ゴーリーの位置を生成
    target_pose = Pose2D(our_goal_pose.x, robot_pose.y, 0)
    target_pose.x += MARGIN_DIST_X
    if button_lb:
        target_pose.y += 0.1
    if button_rb:
        target_pose.y -= 0.1

    # 行き過ぎを抑制する
    if our_goal_upper < target_pose.y:
        target_pose.y = our_goal_upper
    elif target_pose.y < our_goal_lower:
        target_pose.y = our_goal_lower
        
    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

# 特定のボタンを押すとゴール中心へ移動
def move_goal_center(target_id):

    global our_goal_pose

    target_pose = our_goal_pose

    control_target = ControlTarget()
    control_target.robot_id = target_id
    control_target.control_enable = True
    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

# コントローラを離しているときは停止
def stop(target_id):
    global robot_pose    

    target_pose = robot_pose

    control_target = ControlTarget()
    control_target.robot_id = target_id
    control_target.control_enable = True
    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

def RobotPose(data):
    global robot_pose
    robot_pose = data.pose

# フィールドの情報取得
def get_field_info(data):
    global our_goal_pose, our_goal_upper, our_goal_lower

    field_length = data.field_length
    field_width  = data.field_width

    # ゴール幅
    our_goal_upper = 0.5
    our_goal_lower = -0.5
    # our_goal_upper = data.goal_width/2
    # our_goal_lower = -data.goal_width/2

    # ゴール座標
    our_goal_pose.x = -field_length/2
    our_goal_pose.y = 0


def main():

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
    sub_robot = rospy.Subscriber(topic_name_robot_info, RobotInfo, RobotPose)

    # フィールドの情報をもらう
    sub_vision = rospy.Subscriber(topic_vision, VisionGeometry, get_field_info)

    # joystick
    joy_wrapper = joystick_example.JoyWrapper()

    rospy.sleep(3.0)

    # 制御目標値を生成
    r = rospy.Rate(60)

    # 制御目標値を生成
    button_flag = 0
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

        if button_lb or button_rb:
            # パスの生成
            control_target = make_path(
                                    TARGET_ID,
                                    joy_wrapper,
                                    button_lb,
                                    button_rb,
                                )
        elif button_x:
            control_target = move_goal_center(TARGET_ID)
        else:
            control_target = stop(TARGET_ID)

        pub.publish(control_target)

        r.sleep()

if __name__ == '__main__':
    main()
