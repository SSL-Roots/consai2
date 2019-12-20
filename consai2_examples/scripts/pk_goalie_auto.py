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

current_dir = os.path.dirname(__file__)
game_dir = os.path.join(
                    current_dir,
                    '../../consai2_game/scripts/example'
                )
sys.path.append(game_dir)
from actions import goalie, tool

# ボール情報
ball_info = BallInfo()
# ロボット情報
robot_info = {'our':[],'their':[]}
# ゴール情報
our_goal_pose = Pose2D()
our_goal_upper = 0
our_goal_lower = 0

# ゴール前までの距離[m]
MARGIN_DIST_X = 0.1

# ボタン操作で目標に生成する移動距離
MOVE_DISTANCE = 0.1

# 直線の傾きと切片を算出
# 2点の座標から算出する
def _get_line_parameters(pose1, pose2):

    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y

    # 0になるとエラーになるのでその対策
    if x1 - x2 == 0:
        x1 += 1e-12
    
    # 傾きの算出
    slope = (y2 - y1) / (x2 - x1)
    # 切片の算出
    intercept = y2 - slope * x2
    
    return slope, intercept

# AIゴーリーU/
def kill_mode(control_target):
    # 制御目標値を生成
    global robot_info, ball_info, our_goal_pose, our_goal_upper, our_goal_lower
    global MARGIN_DIST_X, MOVE_DISTANCE

    # Trueで走行開始
    control_target.control_enable = True

    # ボールが動いていると判断するしきい値[m/s]
    MOVE_BALL_VELOCITY_THRESHOLD = 0.5
    # ロボットがボールを持っていると判断するしきい値[m]
    DIST_ROBOT_TO_BALL_THRESHOLD = 0.2
    # ゴールライン上ではなく一定距離[m]前を守るための変数
    MARGIN_DIST_X = -0.1

    # ゴールの位置(自陣のゴールのx座標は絶対負になるのは過去の話)
    OUR_GOAL_POSE = our_goal_pose
    OUR_GOAL_UPPER = our_goal_upper
    OUR_GOAL_LOWER = our_goal_lower

    # 敵ロボットの位置
    robot_pose = robot_info['their'][0].pose

    # ボールの位置
    ball_pose = ball_info.pose

    # ボールとロボットの距離
    dist = tool.distance_2_poses(robot_pose, ball_pose)
    # ボールの速度
    ball_velocity_x = ball_info.velocity.x
    ball_velocity_y = ball_info.velocity.y
    ball_velocity = math.hypot(ball_velocity_x, ball_velocity_y)

    # ボールの進む角度
    angle_ball = math.atan2(ball_velocity_y, ball_velocity_x)
    # ボールの進む変化量を計算（方向を考慮した単位量）
    var_ball_velocity_x = math.cos(angle_ball) 
    var_ball_velocity_y = math.sin(angle_ball) 

    # ボールの次の予測位置を取得
    ball_pose_next = Pose2D(
            ball_pose.x + var_ball_velocity_x, ball_pose.y + var_ball_velocity_y, 0) 

    # 敵ロボットとボールの距離が近い場合は敵の向いている直線を使う
    if dist < DIST_ROBOT_TO_BALL_THRESHOLD and robot_pose.x > 0:
        slope = math.tan(robot_pose.theta)
        intercept = robot_pose.y - slope * robot_pose.x 

    # ボールの速度がある場合かつ近づいてくる場合はボールの向かう直線を使う
    elif MOVE_BALL_VELOCITY_THRESHOLD < ball_velocity and ball_velocity_x > 0:
        slope, intercept = _get_line_parameters(ball_pose, ball_pose_next)

    # その他はゴール中心とボールを結ぶ直線を使う
    else:
        slope, intercept = _get_line_parameters(ball_pose, OUR_GOAL_POSE)

    # ゴーリの新しい座標
    goalie_pose_x = OUR_GOAL_POSE.x + MARGIN_DIST_X
    goalie_pose_y = slope*goalie_pose_x + intercept

    # ゴールから飛び出さないようる上下限を設ける
    if OUR_GOAL_UPPER < goalie_pose_y:
        goalie_pose_y = OUR_GOAL_UPPER
    elif goalie_pose_y < OUR_GOAL_LOWER:
        goalie_pose_y = OUR_GOAL_LOWER

    # ゴーリの新しい座標
    new_goalie_pose = Pose2D(goalie_pose_x, goalie_pose_y, math.pi)

    control_target.path = []
    control_target.path.append(new_goalie_pose)

    return control_target

# ボールの情報を取得
def get_ball_info(msg):
    global ball_info
    ball_info = msg

# ロボットの情報を取得
def our_robot_info(msg, robot_id):
    global robot_info
    robot_info['our'][robot_id] = msg

# ロボットの情報を取得
def their_robot_info(msg, robot_id):
    global robot_info
    robot_info['their'][robot_id] = msg

# フィールドの情報取得
def get_field_info(data):
    global our_goal_pose, our_goal_upper, our_goal_lower
    
    # フィールド長
    field_length = data.field_length

    # ゴール幅
    our_goal_upper = data.goal_width/2
    our_goal_lower = -data.goal_width/2

    # ゴール座標
    our_goal_pose.x = field_length/2
    our_goal_pose.y = 0


def main():
    
    global robot_info, ball_info

    rospy.init_node('control_example')
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    THEIR_COLOR = rospy.get_param('~color', 'blue')

    # アタッカ＝の反対のcolorをゴーリーにセットする
    COLOR = 'yellow' # 'blue' or 'yellow'
    if THEIR_COLOR == 'yellow':
        COLOR = 'blue'

    # COLOR = "blue" # 'blue' or 'yellow'
    # THEIR_COLOR = "yellow" # 'blue' or 'yellow'

    TARGET_ID = 0 # 0 ~ MAX_ID

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]

    # print sub
    topic_name = 'consai2_game/control_target_' + COLOR + '_' + topic_id
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    # フィールドの情報をもらう
    # topic_vision = COLOR + SIDE + '/vision_receiver/raw_vision_geometry'
    topic_vision = 'vision_receiver/raw_vision_geometry'
    sub_vision = rospy.Subscriber(topic_vision, VisionGeometry, get_field_info)

    # ボールの位置を取得
    sub_ball_info = rospy.Subscriber(
            'vision_wrapper/ball_info', BallInfo,
            get_ball_info, queue_size=1)

    # ロボットの位置
    robot_info['our'].append(RobotInfo())
    robot_info['their'].append(RobotInfo())

    topic_name = 'vision_wrapper/robot_info_' + COLOR + '_' + topic_id
    sub_robot_info_our = rospy.Subscriber(topic_name, RobotInfo, 
            our_robot_info,
            callback_args=TARGET_ID)

    topic_name = 'vision_wrapper/robot_info_' + THEIR_COLOR + '_' + topic_id
    sub_robot_info_there = rospy.Subscriber(topic_name, RobotInfo, 
            their_robot_info,
            callback_args=TARGET_ID)

    # CotrolTargetの生成
    control_target = ControlTarget()
    # ロボットIDを設定
    control_target.robot_id = TARGET_ID

    # 制御目標値を生成
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        control_target = kill_mode(control_target)
        # control_target = goalie.interpose(ball_info, robot_info, control_target)
        pub.publish(control_target)
        r.sleep()

if __name__ == '__main__':
    main()


