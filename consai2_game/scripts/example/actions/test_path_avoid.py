# coding: UTF-8

# defense.pyでは、ボールを蹴らないactionを定義する

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool
import path_avoid

sys.path.append(os.pardir)
from field import Field


def interpose(my_pose, target_info, robot_info, control_target, 
        dist_from_goal=None, dist_from_target=None):
    # 自チームのゴール中心とtarget_info.poseを直線で結び、その直線上に移動する
    # dist_from_goal is not Noneなら、ゴール中心からdist分離れた位置に移動する
    # dist_from_target is not Noneなら、target_info.poseからdist分離れた位置に移動する


    # 到達姿勢の計算とcontrol_targetの更新(path以外)
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    # 両方設定されてなかったらdist_from_targetを優先する
    if dist_from_goal is None and dist_from_target is None:
        dist_from_target = 0.6 # 適当な値

    OUR_GOAL_POSE = Field.goal_pose('our', 'center')
    angle_to_target = tool.get_angle(OUR_GOAL_POSE, target_info.pose)

    new_goal_pose = Pose2D()
    if dist_from_goal is not None:
        trans = tool.Trans(GOAL_POSE, angle_to_target)
        tr_goal_pose = Pose2D(dist_from_goal, 0, 0)
        new_goal_pose = trans.inverted_transform(tr_goal_pose)
    else:
        angle_to_goal = tool.get_angle(target_info.pose, OUR_GOAL_POSE)
        trans = tool.Trans(target_info.pose, angle_to_goal)
        tr_goal_pose = Pose2D(dist_from_target, 0, 0)
        new_goal_pose = trans.inverted_transform(tr_goal_pose)

    new_goal_pose.theta = angle_to_target

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

    
    # ---------------------------------------------------------
    # 中間パスの生成(毎フレーム算出する)
    avoid_pose = path_avoid.basic_avoid(my_pose, target_info.pose, new_goal_pose, robot_info, angle_to_goal)

    # 現在のパスの長さ
    l = len(control_target.path)
    if 0 < l:
        goal_pose = control_target.path[-1]
        control_target.path = []
        if avoid_pose is not None:
            control_target.path.append(avoid_pose)
            control_target.path.append(goal_pose)
        else:
            control_target.path.append(goal_pose)

    # ---------------------------------------------------------
    # remake_path is Trueならpathを再生成する
    # pathを再生成すると衝突回避用に作られた経路もリセットされる
    if remake_path:
        control_target.path = []
        # 移動経路上にロボットが居たら回避するパスを生成する

        if avoid_pose is not None:
            control_target.path.append(avoid_pose)
        control_target.path.append(new_goal_pose)

    return control_target, True
    # return control_target, remake_path


