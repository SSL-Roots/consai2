#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo, RobotInfo

import math

ball_pose = Pose2D()
target_pose = Pose2D()
ball_vel = None

xg = -6
yg = 0
# 桑田邸
# xr = -0.8
# grsim
xr = -5.7

goal_pose = Pose2D()
goal_pose.x = xg 
goal_pose.y = yg 

goalie_threshold_y = 1.5

def set_pose(control_target, target_id):
    
    global ball_pose, goal_pose, ball_vel

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
        dt = 1/60
        vx = ball_vel.x
        vy = ball_vel.y
        va = ball_vel.theta
        v = math.hypot(ball_vel.x, ball_vel.y)

        dx = vx * dt
        dy = vx * dt
        da = va * dt

    # ボールの現在位置を取得
    xb = ball_pose.x
    yb = ball_pose.y

    # ボールの次の予測位置を取得
    xb_n = xb + dx
    yb_n = yb + dy
    ball_pose_next = Pose2D() 
    ball_pose_next.x = xb + xb_n
    ball_pose_next.y = yb + yb_n

    # ボールの位置の差分
    d = distance_2_poses(ball_pose, ball_pose_next)
    # print d

    # ボールが動いていないとき
    if v < 0.05:
        a, b = line_slope_and_gradient(ball_pose, goal_pose)
    else:
        a, b = line_slope_and_gradient(ball_pose, ball_pose_next)
    
    # 位置を決める
    yr = a*xr + b
    # 位置
    target_pose.x = xr
    target_pose.y = yr
    
    if target_pose.y > goalie_threshold_y:
        target_pose.y = goalie_threshold_y
    elif target_pose.y < -goalie_threshold_y:
        target_pose.y = -goalie_threshold_y

    # 移動する
    control_target.path.append(target_pose)

    return control_target

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

    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    print 'control_exmaple start'
    rospy.sleep(3.0)

    # ballの位置を取得する
    sub = rospy.Subscriber('vision_wrapper/ball_info', BallInfo, get_ball_pose)

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
