# coding: UTF-8

# offense.pyでは、ボールを蹴るactionを定義する

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool

sys.path.append(os.pardir)
from field import Field


def simple_kick(my_pose, ball_info, control_target, kick_power=0.5):
    # ボールへまっすぐ向かい、そのまま蹴る
    # シュート目標位置を設定しないシンプルなaction

    
    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    angle_robot_to_ball = tool.get_angle(my_pose, ball_info.pose)
    trans = tool.Trans(ball_info.pose, angle_robot_to_ball)
    tr_position= Pose2D(0.2, 0, 0) # ボールより少し前を目標位置にする
    position = trans.inverted_transform(tr_position)

    new_goal_pose = Pose2D()
    new_goal_pose = position
    new_goal_pose.theta = angle_robot_to_ball

    # ボールに近づいたらキックフラグをON
    if tool.distance_2_poses(my_pose, ball_info.pose) < 0.5:
        control_target.kick_power = kick_power
    else:
        control_target.kick_power = 0.0

    # ドリブルは常にオフ
    control_target.dribble_power = 0.0

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


def setplay_shoot(my_pose, ball_info, control_target, kick_enable=False):
    # セットプレイ用のシュートアクション
    # kick_enable is Falseで、ボールの近くまで移動する
    # kick_enable is True で、シュートする
    # デフォルトでゴールを狙う

    KICK_POWER = 0.8
    SHOOT_TARGET = Field.goal_pose('their', 'center')

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    angle_ball_to_target = tool.get_angle(ball_info.pose, SHOOT_TARGET)
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_my_pose = trans.transform(my_pose)

    # ロボットがボールの裏側に回ったらcan_kick is True
    can_kick = False
    if tr_my_pose.x < 0.01 and math.fabs(tr_my_pose.y) < 0.05:
        can_kick = True
    

    avoid_ball = True # ボールを回避する
    new_goal_pose = Pose2D()
    if can_kick and kick_enable:
        # ボールをける
        avoid_ball = False

        # ボールの前方に移動する
        new_position = trans.inverted_transform(Pose2D(0.02, 0, 0))
        new_goal_pose = new_position
        new_goal_pose.theta = angle_ball_to_target
        # ドリブルとキックをオン
        control_target.kick_power = KICK_POWER
    else:
        # ボールの裏に移動する
        new_position = trans.inverted_transform(Pose2D(-0.3, 0, 0))
        new_goal_pose = new_position
        new_goal_pose.theta = angle_ball_to_target
        # ドリブルとキックをオフ
        control_target.kick_power = 0.0
        control_target.dribble_power = 0.0
        
    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)


    return control_target, avoid_ball


