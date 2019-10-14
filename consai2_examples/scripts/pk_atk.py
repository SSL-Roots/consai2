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

ball_info = BallInfo()
robot_info = RobotInfo()
ball_get_pose = Pose2D()

# ボールを取得したかどうかのFlag
ball_get_state = 1

# パスの生成
def make_path(control_target, joy_wrapper, coordinate, kick_enable, ang_1, ang_2):
    # 制御目標値を生成
    global robot_info, ball_info, ball_get_state, ball_get_pose

    # ボールとロボットの位置を取得
    robot_pose = robot_info.pose
    ball_pose = ball_info.pose

    # 新しいパスを格納する変数
    target_pose = Pose2D()
    # target_pose = robot_pose

    # 角度調整用
    robot_commands = RobotCommands()
    robot_commands.header.stamp = rospy.Time.now()

    command = RobotCommand()

    # Trueで走行開始
    control_target.control_enable = True

    # 経路生成
    coordinate._update_approach_to_shoot()
    # ロボットとボールの間の距離
    dist = distance_2_poses(robot_pose, ball_pose)
    # ロボットとボール間の相対角度(degで取得)
    angle_rb = angle_2_diff(robot_pose.theta, angle_2_poses(robot_pose, ball_pose), unit='deg')

    dist_th = 0.20
    ang_th = 30

    kick_flag = False
    control_target.dribble_power = 0.0
    control_target.kick_power = 0.0
    # ボールがしきい値内かどうか判定して各動作を生成
    if dist < dist_th and abs(angle_rb) < ang_th:
        # とりあえずドリブルする
        control_target.dribble_power = 0.5
        # ボールの少し前に前進する
        if ball_get_state == 1:
            # ボール位置まで移動してボールを確実に保持する
            target_pose.x = ball_pose.x
            target_pose.y = ball_pose.y
            target_pose.theta = angle_2_poses(robot_pose, ball_pose)

            ball_get_pose = target_pose
            ball_get_state = 2

        if ball_get_state == 2:
            target_pose = ball_get_pose
            if distance_2_poses(robot_pose, ball_get_pose) < 0.1:
                ball_get_state = 3
        else:
            # 角度調整
            # command.robot_id = control_target.robot_id
            # command.dribble_power = 0.5
            # command.vel_surge = 0 
            # command.vel_sway = 0
            target_pose = robot_pose
            if ang_1:
                target_pose.theta = target_pose.theta + math.pi * 0.2
                # command.vel_angular = math.pi * 1.0
            elif ang_2:
                target_pose.theta = target_pose.theta - math.pi * 0.2
                # command.vel_angular = -math.pi * 1.0

            # 判定角度以内 + ボタン入力がある場合蹴る
            if abs(angle_rb) < 20 and kick_enable:
                # command.kick_power = 0.3
                control_target.kick_power = 0.3
                ball_get_state = 1
                kick_flag = True

            # robot_commands.commands.append(copy.deepcopy(command))
            # joy_wrapper._pub_commands.publish(robot_commands)

    # しきい値内で無い場合経路生成してボールまで行く
    else:
        ball_get_state = 1
        control_target.dribble_power = 0.0
        control_target.kick_power = 0.0
        target_pose = coordinate.get_target_pose()

    # print(ball_get_state)

    control_target.path = []
    # if ball_get_state != 3:
    control_target.path.append(target_pose)

    return control_target, kick_flag

# コントローラを離しているときは停止
def stop(control_target):
    global robot_info, ball_get_state

    control_target.control_enable = True
    control_target.path = []
    control_target.path.append(robot_info.pose)

    if ball_get_state == 1 or ball_get_state == 2:
        control_target.dribble_power = 0.0
        control_target.kick_power = 0.0

    return control_target


def get_ball_info(msg):
    global ball_info
    ball_info = msg


def get_robot_info(msg):
    global robot_info
    robot_info = msg


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

# ２点間の直線と水平線との角度を計算
def angle_2_poses(pose1, pose2):
    diff_pose = Pose2D()

    diff_pose.x = pose2.x - pose1.x
    diff_pose.y = pose2.y - pose1.y

    return math.atan2(diff_pose.y, diff_pose.x)


# 2点間の距離を計算する
def distance_2_poses(pose1, pose2):
    diff_pose = Pose2D()

    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return math.hypot(diff_pose.x, diff_pose.y)


def main():
    global ball_info, robot_info, bal_get_state
    
    rospy.init_node('control_example')
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    TARGET_ID = 0 # 0 ~ MAX_ID
    ATK_COLOR = "yellow" # 'blue' or 'yellow'
    ATK_SIDE = "right"
    GOALIE_COLOR = "blue" # 'blue' or 'yellow'
    GOALIE_SIDE = "left"

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]

    # print sub
    topic_name = ATK_COLOR + ATK_SIDE + '/consai2_game/control_target_' + ATK_COLOR +'_' + topic_id
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)
    pub_joy = rospy.Publisher('consai2_examples/joy_target', ControlTarget, queue_size=1)

    # ballの位置を取得する
    sub_ball = rospy.Subscriber(ATK_COLOR + ATK_SIDE +'/vision_wrapper/ball_info', BallInfo, get_ball_info)

    # Robotの位置を取得する
    topic_name_robot_info = ATK_COLOR + ATK_SIDE + '/vision_wrapper/robot_info_' + ATK_COLOR + '_' + topic_id
    sub_robot = rospy.Subscriber(topic_name_robot_info, RobotInfo, get_robot_info)

    # joy_
    joy_wrapper = joystick_example.JoyWrapper()

    # control_target
    control_target = ControlTarget()
    control_target.robot_id = TARGET_ID

    # ボールを取りに行くcoordinateクラス
    _coordinate = coordinate.Coordinate(ATK_COLOR, ATK_SIDE)

    # 制御目標値を生成
    r = rospy.Rate(60)
    kick_flag = False
    button_flag = 0
    while 1:
        if joy_wrapper.get_button_status() != None:
            _joy_msg = joy_wrapper.get_button_status()
            button_lb = _joy_msg.buttons[4]
            button_rb = _joy_msg.buttons[5]
            button_x  = _joy_msg.buttons[0]
            button_a  = _joy_msg.buttons[1]
        else:
            button_lb = 0
            button_rb = 0
            button_x  = 0
            button_a  = 0

        # coordinateのアップデート
        _coordinate._update_robot_pose(robot_info.pose)
        _coordinate._update_ball_pose(ball_info.pose)

        if (button_a or button_lb or button_rb or button_x) and kick_flag == False:
            # パスの生成
            control_target, kick_flag = make_path(
                                                control_target,
                                                joy_wrapper,
                                                _coordinate,
                                                button_x,
                                                button_lb,
                                                button_rb)
            # if ball_get_state != 3:
            pub.publish(control_target)
        # 停止    
        else:
            control_target = stop(control_target)
            pub.publish(control_target)
        # 蹴ったあとに追いかける対策
        if button_flag == 1 and button_x == 0:
            kick_flag = False

        button_flag = button_x

        r.sleep()

if __name__ == '__main__':
    main()
