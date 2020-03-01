#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool
import normal
import offense

sys.path.append(os.pardir)
from field import Field

from observer import Observer

# 後ろに周り込むときの距離
SET_POSE_ADD_X_ATK = 0.3
SET_POSE_ADD_X_RECV = 0.1
# kick_power
KICK_POWER = 0.3
# dorrible_power
DRIBBLE_POWER = 0.6

# ボールがplacementされたとみなされる範囲
# 実際にボールがplacementされたとみなされる範囲は0.15[m]
BALL_PLACE_TRESHOLD = 0.10
# ボールを置きに行く動作に入るときの範囲(ドリブルしない）
BALL_PLACE_AREA_NO_DRIBBLE = 0.3
# ボールに近いと判断する距離
BALL_GET_AREA = 4.0
# ボールが動いていると判断する速度
VEL_THRESHOLD = 0.5

# ボール保持の判定
IS_LOOK_TARGET_ANGLE = 4  # deg
IS_TOUCH_DIST = 0.20

# 侵入禁止範囲
# 0.5m以内に侵入してはだめなので0.5以上にする
BALL_MARGIN_DIST = 0.65

# atkが指定位置に到達したか判定
def atk_arrived_check(tr_my_pose):

    flag = False
    if tr_my_pose.x < 0.05 and math.fabs(tr_my_pose.y) < 0.05 and \
            math.fabs(tr_my_pose.theta) < math.radians(IS_LOOK_TARGET_ANGLE): 
        flag = True

    return flag

# recvが指定距離以内に到達したか判定
def recv_arrived_check(dist):

    flag = False
    if dist < 0.1:
        flag = True
    
    return flag

# 目標座標に対してmargin_valueだけ後ろに座標を生成する
def generating_behind_target_pose(trans, target_pose, margin_value):

    tr_behind_target_pose = trans.transform(target_pose)
    tr_behind_target_pose.x = tr_behind_target_pose.x + margin_value
    tr_behind_target_pose.y = tr_behind_target_pose.y
    behind_target_pose = trans.inverted_transform(tr_behind_target_pose)
    
    return behind_target_pose

# target_poseに距離が一番近いロボットがplacementを行うロボットと推定する
def get_near_robot_id(target_pose, robot_info):

    dist = []
    for our_info in robot_info['our']:
        if our_info.disappeared is False:
            # 各ロボットと指定位置の距離
            dist.append(tool.distance_2_poses(our_info.pose, target_pose)) 
        else:
            # インデックスを揃えるためダミーデータを挿入
            dist.append(100)

    # 一番距離が近いロボットのID
    robot_id = dist.index(min(dist))

    return robot_id

# ball placementrをatkとrecvの2台で行う
def basic_ball_placement(control_target, target_pose, ball_info, robot_info, my_id, mode, field_size):

    # ペアになるロボットのID
    # target_poseに一番距離が一番近いロボットを使う
    if mode == 'atk':
        atk_id = my_id
        recv_id = get_near_robot_id(target_pose, robot_info)
    else:
        atk_id = get_near_robot_id(target_pose, robot_info)
        recv_id = my_id

    # placementを行うロボットの座標
    atk_pose = robot_info['our'][atk_id].pose
    recv_pose = robot_info['our'][recv_id].pose

    # placementを行うための座標などを生成する
    if mode == 'atk':
        control_target, avoid_ball = basic_atk(control_target, ball_info, atk_pose, recv_pose, target_pose, field_size)
    else:
        control_target, avoid_ball = basic_recv(control_target, ball_info, atk_pose, recv_pose, target_pose)
    
    return control_target, avoid_ball

# ball_placement(atkロボット)の動作
def basic_atk(control_target, ball_info, atk_pose, recv_pose, target_pose, field_size):

    # ボールからみた目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, target_pose)
    
    # 目標座標からみたボールまでの角度
    angle_target_to_ball = tool.get_angle(target_pose, ball_info.pose)

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)

    # ボールとpalcement位置の距離
    dist_ball_to_target = tool.distance_2_poses(ball_info.pose, target_pose)

    # ボールとロボットの距離
    dist_ball_to_atk = tool.distance_2_poses(ball_info.pose, atk_pose)

    # recvがボールのレシーブを行う座標
    ball_receiving_pose = generating_behind_target_pose(trans, target_pose, SET_POSE_ADD_X_RECV)

    # ボール速度
    ball_velocity = math.hypot(ball_info.velocity.x, ball_info.velocity.y)

    # 壁際と判定するしきい値
    wall_decision_dist = [0.3, 0.2]

    # フィールドの情報をまとめる(上、下、左、右の位置)
    field_pose = [field_size[0]/2,
                  -field_size[0]/2,
                  field_size[1]/2,
                  -field_size[1]/2]

    # 壁際にあるか判定しておく
    wall_decision_result = [False, False]
    for i, val in enumerate(wall_decision_dist):
        if field_pose[0] - val < ball_info.pose.x or \
                ball_info.pose.x < field_pose[1] + val or \
                field_pose[2] - val < ball_info.pose.y or \
                ball_info.pose.y < field_pose[3] + val:
            wall_decision_result[i] = True

    # ---------------------------------------
    # placementの行動生成

    # デフォルト値
    avoid_ball = False
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    new_pose = Pose2D()
    # ボールが範囲に入っていない場合はplacementを行う
    if BALL_PLACE_TRESHOLD < dist_ball_to_target:

        # 指定位置に到着したかどうか
        is_atk_arrived = atk_arrived_check(trans.transform(atk_pose))
        is_recv_arrived = recv_arrived_check(tool.distance_2_poses(recv_pose, ball_receiving_pose))
        
        # 蹴ったあとにatkが追いかけない様に対策
        if VEL_THRESHOLD < ball_velocity:
            new_pose = atk_pose

        # ボールが壁際にある場合はドリブルしながら下がる
        elif wall_decision_result[0]:
            if wall_decision_result[1]:
                # ボールを保持して下がる
                if dist_ball_to_atk < 0.11:
                    control_target.dribble_power = DRIBBLE_POWER
                    new_pose = trans.inverted_transform(Pose2D(0.2, 0, 0))
                    new_pose.theta = angle_target_to_ball
                # ボールの後ろまで移動する
                else:
                    control_target.dribble_power = 0
                    new_pose = trans.inverted_transform(Pose2D(0.1, 0, 0))
                    new_pose.theta = angle_target_to_ball
            # 壁からある程度離れたらロボットは一度下がる(avoidanceの邪魔にならないように)
            else:
                control_target.dribble_power = 0
                new_pose = trans.inverted_transform(Pose2D(0.4, 0.4, 0))
                new_pose.theta = angle_target_to_ball
        
        # atkがボールの後ろにいなければ移動する
        elif not is_atk_arrived:
            # ボールの後ろに座標を生成
            new_pose = trans.inverted_transform(Pose2D(-SET_POSE_ADD_X_ATK, 0, 0))
            new_pose.theta = angle_ball_to_target
            
            avoid_ball = True

        # もしボールとゴールに近い場合はアタッカーが置きにいく
        elif dist_ball_to_target < BALL_GET_AREA:
            
            # ドリブルする
            control_target = offense.inplay_dribble(atk_pose, ball_info, 
                    control_target,target_pose)
            new_pose = control_target.path[-1]
           
            # ドリブルをやめる範囲ならドリブル
            control_target.dribble_power = 0.0
            if BALL_PLACE_AREA_NO_DRIBBLE < dist_ball_to_target:
                control_target.dribble_power = DRIBBLE_POWER

        # お互いの位置がセットされたら蹴る
        elif is_atk_arrived and is_recv_arrived:

            # ボールを確実に保持するためボールの少し前に移動する
            new_pose = trans.inverted_transform(Pose2D(0.2, 0, 0))
            new_pose.theta = angle_ball_to_target

            # ドリブルとキックをオン
            control_target.kick_power = KICK_POWER
            control_target.dribble_power = DRIBBLE_POWER

        # ボールの後ろに移動
        else:
            new_pose = trans.inverted_transform(Pose2D(-SET_POSE_ADD_X_ATK, 0, 0))
            new_pose.theta = angle_ball_to_target

            avoid_ball = True

    # 範囲内にボールがある場合は離れる
    else:
        # ボールから離れる動作
        angle_atk_to_target = tool.get_angle(target_pose, atk_pose)
        new_pose.x = target_pose.x + BALL_MARGIN_DIST * math.cos(angle_atk_to_target)
        new_pose.y = target_pose.y + BALL_MARGIN_DIST * math.sin(angle_atk_to_target)
        new_pose.theta = angle_atk_to_target + math.pi

    # パスを追加
    control_target.path = []
    control_target.path.append(new_pose)

    return control_target, avoid_ball

# ball_placement(recvロボット)の動作
def basic_recv(control_target, ball_info, atk_pose, recv_pose, target_pose):

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, target_pose)

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)

    # ボールとpalcement位置の距離
    dist_ball_to_target = tool.distance_2_poses(ball_info.pose, target_pose)

    # ボールの速度
    ball_velocity = math.hypot(ball_info.velocity.x, ball_info.velocity.y)

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, target_pose)

    # ---------------------------------------
    # placementの行動生成

    # デフォルト値
    avoid_ball = True
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    new_pose = Pose2D()
    if BALL_PLACE_TRESHOLD < dist_ball_to_target:

        # 速度が出ている場合レシーブしにいく
        if VEL_THRESHOLD < ball_velocity:
            
            # レシーブするための座標生成
            new_pose = receive_ball(ball_info, recv_pose)

            # ドリブルする
            control_target.dribble_power = DRIBBLE_POWER

        # placement座標の少し後ろに移動する
        else:
            new_pose = generating_behind_target_pose(trans, target_pose, SET_POSE_ADD_X_RECV) 
            new_pose.theta = angle_ball_to_target + math.pi
    else:
        # placement後に離れる
        angle_recv_to_target = tool.get_angle(target_pose, recv_pose)
        new_pose.x = target_pose.x + BALL_MARGIN_DIST * math.cos(angle_recv_to_target)
        new_pose.y = target_pose.y + BALL_MARGIN_DIST * math.sin(angle_recv_to_target)
        new_pose.theta = angle_recv_to_target + math.pi

        avoid_ball = False

    # パスを追加
    control_target.path = []
    control_target.path.append(new_pose)

    return control_target, avoid_ball

# 配置を担当しないロボットは避ける
def avoid_ball_place_line(my_pose, ball_info, target_pose, control_target, force_avoid=False):

    # ボール中心に座標変換
    angle_ball_to_target = tool.get_angle(ball_info.pose, target_pose)
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_my_pose = trans.transform(my_pose)
    tr_target_pose = trans.transform(target_pose)
    dist_ball_to_target = tool.distance_2_poses(ball_info.pose, target_pose)

    # ライン上にいるロボットは回避位置を生成する
    # ラインに対して垂直に移動する
    if BALL_PLACE_AREA_NO_DRIBBLE < dist_ball_to_target or force_avoid:
        if -BALL_MARGIN_DIST < tr_my_pose.y < BALL_MARGIN_DIST:
            if tr_my_pose.y < 0:
                tr_my_pose.y -= BALL_MARGIN_DIST
            else:
                tr_my_pose.y += BALL_MARGIN_DIST

    # 避ける位置を生成
    target_pose = trans.inverted_transform(tr_my_pose)

    control_target.path = []
    control_target.path.append(target_pose)

    return control_target, True

# ボール受け取り位置の生成
def receive_ball(ball_info, my_pose):
    ball_pose = ball_info.pose
    _can_receive_dist = 1.0
    _can_receive_hysteresis = 0.3

    target_pose = Pose2D()

    if Observer.ball_is_moving():
        angle_velocity = tool.get_angle_from_center(ball_info.velocity)
        trans = tool.Trans(ball_pose, angle_velocity)

        tr_pose = trans.transform(my_pose)

        fabs_y = math.fabs(tr_pose.y)

        tr_pose.y = 0.0
        inv_pose = trans.inverted_transform(tr_pose)
        angle_to_ball = tool.get_angle(inv_pose, ball_pose)
        target_pose = Pose2D(inv_pose.x, inv_pose.y, angle_to_ball)

    return target_pose
