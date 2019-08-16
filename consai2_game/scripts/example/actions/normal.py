# coding: UTF-8

# normal.pyでは攻撃も防御もしないactionを定義する


import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool

sys.path.append(os.pardir)
from field import Field
import role


def make_line(my_role, ball_info, control_target, 
        start_x=-3, start_y=-3, add_x=0.3, add_y=0):
    # ROLE順に１列に並ぶ

    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    new_goal_pose = Pose2D()
    new_goal_pose.x = start_x + my_role*add_x
    new_goal_pose.y = start_y + my_role*add_y

    # ボールを見る
    angle_to_ball = tool.get_angle(new_goal_pose, ball_info.pose)
    new_goal_pose.theta = angle_to_ball


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

    return control_target, remake_path
    

def stop(control_target):
    # 速度0で停止する

    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    # パスを初期化する
    control_target.path = []

    # 目標速度を0にする
    control_target.goal_velocity = Pose2D(0, 0, 0)


    # ---------------------------------------------------------
    remake_path = False

    return control_target, remake_path


def move_to(control_target, pose, ball_info, look_ball=False):
    # 指定位置に移動する
    # ボールを見ることもできる

    new_goal_pose = pose

    if look_ball:
        new_goal_pose.theta = tool.get_angle(new_goal_pose, ball_info.pose)

    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    # パスを初期化する
    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target


def keep_x(control_target, pose_x, ball_info):
    # x座標をpose_xに、y座標をボールのy座標に合わせて移動する

    new_goal_pose = Pose2D()
    new_goal_pose.x = pose_x
    new_goal_pose.y = ball_info.pose.y
    new_goal_pose.theta = tool.get_angle(new_goal_pose, ball_info.pose)

    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    # パスを初期化する
    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target

