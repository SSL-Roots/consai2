#!/usr/bin/env python2
# coding: UTF-8

import rospy

from consai2_msgs.msg import ControlTarget, RobotCommands, RobotCommand
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo, RobotInfo

import joystick_example
import coordinate

ball_pose = Pose2D()
robot_pose = Pose2D()
target_pose = Pose2D()

# 制御目標値を生成
control_target = ControlTarget()

def path_example(target_id, coordinate, joy_wrapper):
    
    # 制御目標値を生成
    global control_target, target_pose
    control_target = ControlTarget()

    # ロボットID
    control_target.robot_id = target_id
    # Trueで走行開始
    control_target.control_enable = True

    coordinate._update_approach_to_shoot()

    if coordinate.approach_state == 3:
        joy_wrapper.update()

    target_pose = coordinate.get_target_pose()
    control_target.path.append(target_pose)
    return control_target

def BallPose(data):
    global ball_pose
    ball_pose = data.pose

def RobotPose(data):
    global robot_pose
    robot_pose = data.pose


def main():
    rospy.init_node('control_example')
    
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    COLOR = "blue" # 'blue' or 'yellow'
    TARGET_ID = 3 # 0 ~ MAX_ID

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = 'consai2_game/control_target_' + COLOR +'_' + topic_id

    topic_name_robot_info = 'vision_wrapper/robot_info_' + COLOR + '_' + str(TARGET_ID)

    _coordinate = coordinate.Coordinate()

    # print sub
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    # joy_
    joy_wrapper = joystick_example.JoyWrapper()
    print 'control_exmaple start'

    rospy.sleep(3.0)

    # 制御目標値を生成
    r = rospy.Rate(60)
    while 1:
        # ballの位置を取得する
        sub_ball = rospy.Subscriber('vision_wrapper/ball_info', BallInfo, BallPose)
        # Robotの位置を取得する
        sub_robot = rospy.Subscriber(topic_name_robot_info, RobotInfo, RobotPose)

        # joy_wrapper.update()
        lb = joy_wrapper.get_button_status()
        if lb:
            _coordinate._update_robot_pose(robot_pose)
            _coordinate._update_ball_pose(ball_pose)

            # パスの生成
            control_target = path_example(TARGET_ID, _coordinate, joy_wrapper)

            pub.publish(control_target)
        r.sleep()

    print 'control_exmaple finish'

if __name__ == '__main__':
    main()
