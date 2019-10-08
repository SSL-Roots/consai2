#!/usr/bin/env python2
# coding: UTF-8

import rospy
import copy

from consai2_msgs.msg import ControlTarget, RobotCommands, RobotCommand
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo, RobotInfo

import joystick_example
import coordinate

import math

ball_pose = Pose2D()
robot_pose = Pose2D()
target_pose = Pose2D()

# 制御目標値を生成
control_target = ControlTarget()
command = RobotCommand()

# ボールを取得したかどうかのFlag
ball_get_state = 1

def path_example(target_id, coordinate, joy_wrapper, button, ang_vel):
    # 制御目標値を生成
    global control_target, target_pose, command, robot_pose, ball_pose, ball_get_state
    control_target = ControlTarget()

    robot_commands = RobotCommands()
    robot_commands.header.stamp = rospy.Time.now()

    _robot_pose = robot_pose
    _ball_pose = ball_pose

    # ロボットID
    control_target.robot_id = target_id
    # Trueで走行開始
    control_target.control_enable = True

    # 経路生成
    coordinate._update_approach_to_shoot()
    # ロボットとボールの間の距離
    dist = distance_2_poses(_robot_pose, _ball_pose)
    # ロボットとボール間の相対角度(degで取得)
    angle_rb = angle_2_diff(_robot_pose.theta, angle_2_poses(_robot_pose, _ball_pose), unit='deg')

    dist_th = 0.13
    ang_th = 30

    kick_flag = False
    # ボールがしきい値内かどうか判定
    # print(dist, angle_rb)
    if dist < dist_th and abs(angle_rb) < ang_th:
        # print(ball_get_state, dist, angle_rb)
        if ball_get_state == 1:
            ang = angle_2_poses(_robot_pose, _ball_pose)
            dx = 0.05 * math.cos(ang*math.pi/180)
            dy = 0.05 * math.sin(ang*math.pi/180)
            target_pose.x = ball_pose.x + dx
            target_pose.y = ball_pose.y + dy
            control_target.path.append(target_pose)
            ball_get_state = 2
            print('1')
           
        elif ball_get_state == 2: 

            control_target.dribble_power = 0
            control_target.path.append(target_pose)
            if distance_2_poses(_robot_pose, target_pose) < 0.03:
                ball_get_state = 3
            print('2')
        else:
            # 角度調整
            command.robot_id = target_id
            command.vel_surge = 0.1
            command.vel_sway = 0
            command.vel_angular = ang_vel * math.pi * 0.25

            command.dribble_power = 1.0
            # 指定角度以内 + ボタン入力がある場合蹴る
            if abs(angle_rb) < 20 and button:
                command.kick_power = 0.3
                ball_get_state = 1
                kick_flag = True
            else:
                command.kick_power = 0
            print('3')

            robot_commands.commands.append(copy.deepcopy(command))
            joy_wrapper._pub_commands.publish(robot_commands)

    # しきい値内では無い場合経路生成してボールまで行く
    else:
        ball_get_state = 1

        control_target.dribble_power = 0
        control_target.kick_power = 0
        target_pose = coordinate.get_target_pose()
        control_target.path.append(target_pose)
        print('4')

    return control_target, kick_flag

# コントローラを離しているときは停止
def stop(target_id):
    
    global control_target
    
    control_target.robot_id = target_id
    control_target.control_enable = False
    
    return control_target


def BallPose(data):
    global ball_pose
    ball_pose = data.pose


def RobotPose(data):
    global robot_pose
    robot_pose = data.pose


# 2つの角度の差分を取る
def angle_2_diff(angle_1, angle_2, unit='rad'):
    angle_diff = (angle_1 - angle_2) 
    angle_diff = angle_normalize(angle_diff)

    # 単位をdegに直す
    if unit == 'deg':
        angle_diff *= 180/math.pi

    return angle_diff


# 角度を-pi ~ piに変換
def angle_normalize(angle):
    while angle > math.pi:
        angle -= 2*math.pi

    while angle < -math.pi:
        angle += 2*math.pi

    return angle


# 角度を計算する
# ２点間の直線と水平線との角度
def angle_2_poses(pose1, pose2):
    diff_pose = Pose2D()

    diff_pose.x = pose2.x - pose1.x
    diff_pose.y = pose2.y - pose1.y

    return math.atan2(diff_pose.y, diff_pose.x)


# 2点間の距離を計算する
def distance_2_poses(pose1, pose2):
    # 2点間の距離を取る
    # pose.theta は使用しない

    diff_pose = Pose2D()

    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return math.hypot(diff_pose.x, diff_pose.y)


def main():
    rospy.init_node('control_example')
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    COLOR = "blue" # 'blue' or 'yellow'
    TARGET_ID = 1 # 0 ~ MAX_ID
    SIDE = "left"

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = COLOR + SIDE + '/consai2_game/control_target_' + COLOR +'_' + topic_id

    topic_name_robot_info = COLOR + SIDE + '/vision_wrapper/robot_info_' + COLOR + '_' + str(TARGET_ID)

    _coordinate = coordinate.Coordinate()

    # print sub
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)
    pub_joy = rospy.Publisher('consai2_examples/joy_target', ControlTarget, queue_size=1)

    # ballの位置を取得する
    sub_ball = rospy.Subscriber(COLOR + SIDE +'/vision_wrapper/ball_info', BallInfo, BallPose)
    # Robotの位置を取得する
    sub_robot = rospy.Subscriber(topic_name_robot_info, RobotInfo, RobotPose)

    # joy_
    joy_wrapper = joystick_example.JoyWrapper()
    print 'control_exmaple start'

    rospy.sleep(3.0)

    # 制御目標値を生成
    r = rospy.Rate(60)

    kick_flag = False
    button_flag = 0
    while 1:
        if joy_wrapper.get_button_status() != None:
            _joy_msg = joy_wrapper.get_button_status()
            button_lb = _joy_msg.buttons[4]
            button_x  = _joy_msg.buttons[0]
            ang_vel   = _joy_msg.axes[3] 
        else:
            button_lb = 0
            button_x  = 0
            ang_vel   = 0

        _coordinate._update_robot_pose(robot_pose)
        _coordinate._update_ball_pose(ball_pose)

        if button_lb and kick_flag == False:
            # パスの生成
            control_target, kick_flag = path_example(
                                                TARGET_ID,
                                                _coordinate,
                                                joy_wrapper,
                                                button_x,
                                                ang_vel)

            pub.publish(control_target)
            
        elif button_flag == 1 and button_lb == 0:
            kick_flag = False
        else:
            control_target = stop(TARGET_ID)
            pub.publish(control_target)
            pub_joy.publish(control_target)

        button_flag = button_lb

        r.sleep()

    print 'control_exmaple finish'


if __name__ == '__main__':
    main()
