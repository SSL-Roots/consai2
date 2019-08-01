#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo

import math

ball_pose = Pose2D()
ball_vel = None
goal_pose = Pose2D()
target_pose = Pose2D()

goalie_threshold_y = 0

# ゴールの位置とゴーリーの普段いるx座標を指定
# visionから正式な値はいただく
xg = 0
yg = 0
xr = 0


def set_pose(control_target, target_id):
    
    # ロボットID
    control_target.robot_id = target_id
    # Trueで走行開始
    control_target.control_enable = True

    if ball_vel is None:
        v = 0
        dx = 0
        dy = 0
        da = 0
    else:
        # ボールの速度
        vx = ball_vel.x
        vy = ball_vel.y
        v = math.hypot(ball_vel.x, ball_vel.y)
        # ボールの進む角度
        angle_ball = math.atan2(vy, vx)
        # ボールの変化量を計算（正確ではなく方向を考慮した単位量）
        dx = math.cos(angle_ball) 
        dy = math.sin(angle_ball) 

    # ボールの次の予測位置を取得
    ball_pose_next = Pose2D() 
    ball_pose_next.x = ball_pose.x + dx
    ball_pose_next.y = ball_pose.y + dy

    # ボールが動いていて、ゴールに向かってくる場合
    if 0.05 < v and 0 < dx:
        a, b = line_slope_and_gradient(ball_pose, ball_pose_next)
    else:
        a, b = line_slope_and_gradient(ball_pose, goal_pose)
    
    # 位置を決める
    yr = a*xr + b

    # 位置
    if yr > goalie_threshold_y:
        yr = goalie_threshold_y
    elif yr < -goalie_threshold_y:
        yr = -goalie_threshold_y
    elif ball_pose.x < xg:
        # ボールが後ろに出たらy座標は0にする
        yr = 0

    target_pose.x = xr
    target_pose.y = yr

    # 移動する
    control_target.path.append(target_pose)

    return control_target

# フィールドの情報取得
def get_field_info(data):
    global xg, yg, xr, goal_pose, goalie_threshold_y 
    field_length = data.field_length
    field_width  = data.field_width

    goal_width = data.goal_width 
    goal_depth = data.goal_depth 

    # ゴールのx座標
    xg = - field_length/2 - goal_depth
    # ゴーリーの普段居るx座標
    xr = xg + goal_depth  + 0.3

    goal_pose.x = xg 
    goal_pose.y = 0 

    # ゴールの幅
    goalie_threshold_y = field_width/6

# ボールの位置取得
def get_ball_pose(data):
    global ball_pose, ball_vel
    ball_pose = data.pose
    ball_vel = data.velocity


# 直線の傾きと接点を算出
def line_slope_and_gradient(pose1, pose2):

    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y

    if x1 - x2 == 0:
        x1 += 0.001

    a = (y2 - y1)/(x2 - x1)
    b = y2 - a * x2
    
    return a, b


# 2点間の距離を計算する
def distance_2_poses(pose1, pose2):
    # 2点間の距離を取る
    # pose.theta は使用しない

    diff_pose = Pose2D()

    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return math.hypot(diff_pose.x, diff_pose.y)

# 角度を計算する
# ２点間の直線と水平線との角度
def angle_2_poses(pose1, pose2):
    diff_pose = Pose2D()

    diff_pose.x = pose2.x - pose1.x
    diff_pose.y = pose2.y - pose1.y

    return math.atan2(diff_pose.y, diff_pose.x)



def main():
    rospy.init_node('control_example')
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    COLOR = "blue" # 'blue' or 'yellow'
    TARGET_ID = 5 # 0 ~ MAX_ID

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = 'consai2_game/control_target_' + COLOR +'_' + topic_id

    # 制御の情報
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    # ballの位置を取得する
    sub = rospy.Subscriber('vision_wrapper/ball_info', BallInfo, get_ball_pose)

    # フィールドの情報をもらう
    sub = rospy.Subscriber('vision_receiver/raw_vision_geometry', VisionGeometry, get_field_info)

    print 'control_exmaple start'
    rospy.sleep(3.0)


    # 制御に使うインスタンス生成
    control_target = ControlTarget()

    # 制御目標値を生成
    r = rospy.Rate(60)
    while 1:
        control_target = set_pose(control_target, TARGET_ID)
        pub.publish(control_target)
        r.sleep()

    print 'control_exmaple finish'


if __name__ == '__main__':
    main()
