# coding: UTF-8

# defense.pyでは、ボールを蹴らないactionを定義する

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool

sys.path.append(os.pardir)
from field import Field


# 直線の傾きと接点を算出
def line_pram(pose1, pose2):

    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y

    if x1 - x2 == 0:
        x1 += 0.001

    a = (y2 - y1)/(x2 - x1)
    b = y2 - a * x2
    
    return a, b

# 角度を計算する
# ２点間の直線と水平線との角度
def angle_2_poses(pose1, pose2):
    diff_pose = Pose2D()

    diff_pose.x = pose2.x - pose1.x
    diff_pose.y = pose2.y - pose1.y

    return math.atan2(diff_pose.y, diff_pose.x)

def interpose(ball_info, robot_info, control_target):

    # ボールの位置
    ball_pose = ball_info.pose
    # ボールの速度
    ball_vel = ball_info.velocity

    # ゴールの位置
    OUR_GOAL_POSE = Field.goal_pose('our', 'center')

    xr = OUR_GOAL_POSE.x + 0.3
    goalie_threshold_y = 1.5

    # 敵のロボットの情報
    robot_info_their = robot_info['their']
    dist = []
    for their_info in robot_info_their:
        if their_info.detected:
            dist.append(tool.distance_2_poses(their_info.pose, ball_pose))
        else:
            dist.append(100)

    # 一番近い敵ロボットのID
    min_dist_id = dist.index(min(dist))

    # 一番近い敵ロボットの座標
    robot_pose = robot_info_their[min_dist_id].pose

    # ボールの速度
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
        dvx = math.cos(angle_ball) 
        dvy = math.sin(angle_ball) 

    # ボールの次の予測位置を取得
    ball_pose_next = Pose2D(ball_pose.x + dvx, ball_pose.y + dvy, 0) 


    if dist[min_dist_id] < 0.15 and robot_pose.x < 0:
        rospy.logdebug('their')
        angle_their = robot_pose.theta
        if angle_their == 0:
            a = 1e-10
        else:
            a = math.tan(angle_their)
        b = robot_pose.y - a * robot_pose.x 
    elif 0.02 < v and dvx < 0:
        rospy.logdebug('ball move')
        a, b = line_pram(ball_pose, ball_pose_next)
    else:
        rospy.logdebug('ball stop')
        a, b = line_pram(ball_pose, OUR_GOAL_POSE)
    
    # 位置を決める
    yr = a*xr + b

    # 位置
    if yr > goalie_threshold_y:
        yr = goalie_threshold_y
    elif yr < -goalie_threshold_y:
        yr = -goalie_threshold_y
    # elif ball_pose.x < xr:
        # ボールが後ろに出たらy座標は0にする
        # yr = 0
    new_goal_pose = Pose2D(xr, yr, 0)

    # ---------------------------------------------------------
    remake_path = False
    # pathが設定されてなければpathを新規作成
    if control_target.path is None or len(control_target.path) == 0:
        remake_path = True
    # 現在のpathゴール姿勢と、新しいpathゴール姿勢を比較し、path再生成の必要を判断する
    if remake_path is False:
        current_goal_pose = control_target.path[-1]

        if not tool.is_close(current_goal_pose, new_goal_pose, Pose2D(0.1, 0.1, math.radians(10))):
            remake_path = True
    # remake_path is Trueならpathを再生成する
    # pathを再生成すると衝突回避用に作られた経路もリセットされる
    if remake_path:
        control_target.path = []
        control_target.path.append(new_goal_pose)
    # print(goalie_pose.x, goalie_pose.y, ball_pose.x, ball_pose.y)

    return control_target

