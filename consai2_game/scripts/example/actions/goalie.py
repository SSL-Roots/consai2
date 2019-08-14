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
        x1 += 1e-12

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

    # ボールが動いていると判断するしきい値
    MOVE_V_THRESHOLD = 0.5
    # ロボットがボールを持っていると判断するしきい値
    DIST_ROBOT2BALL_THRESHOLD = 0.2

    # ボールの位置
    ball_pose = ball_info.pose
    # ボールの速度
    ball_vel = ball_info.velocity

    # ゴールの位置(自陣のゴールのx座標は絶対負になる)
    OUR_GOAL_POSE = Field.goal_pose('our', 'center')
    OUR_GOAL_UPPER = Field.goal_pose('our', 'upper')
    OUR_GOAL_LOWER = Field.goal_pose('our', 'lower')

    # ゴールを守るx座標
    xr = OUR_GOAL_POSE.x + 0.1

    # 敵のロボットの情報
    robot_info_their = robot_info['their']
    # 敵ロボットとボールの距離
    dist = []
    for their_info in robot_info_their:

        if their_info.disappeared is False:
            dist.append(tool.distance_2_poses(their_info.pose, ball_pose))
        else:
            dist.append(100)

    # 一番近い敵ロボットのID
    min_dist_id = dist.index(min(dist))

    # 一番近い敵ロボットの座標
    robot_pose = robot_info_their[min_dist_id].pose

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

    if dist[min_dist_id] < DIST_ROBOT2BALL_THRESHOLD and robot_pose.x < 0:
        # 敵ロボットとボールの距離が近い場合
        rospy.logdebug('their')
        angle_their = robot_pose.theta
        if angle_their == 0:
            a = 1e-12
        else:
            a = math.tan(angle_their)
        b = robot_pose.y - a * robot_pose.x 
    elif MOVE_V_THRESHOLD < v and dvx < 0:
        # ボールの速度がある場合かつ近づいてくる場合
        rospy.logdebug('ball move')
        a, b = line_pram(ball_pose, ball_pose_next)
    else:
        # その他はゴール中心とボールを結ぶ線上を守る
        rospy.logdebug('ball stop')
        a, b = line_pram(ball_pose, OUR_GOAL_POSE)
    
    # 位置を決める
    yr = a*xr + b

    # 位置
    if OUR_GOAL_UPPER.y < yr:
        yr = OUR_GOAL_UPPER.y
    elif yr < OUR_GOAL_LOWER.y:
        yr = OUR_GOAL_LOWER.y

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

    return control_target

