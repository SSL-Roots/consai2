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

# 後ろに周り込むときの位置取り
SET_POSE_ADD_X = 0.3
SET_POSE_ADD_X_RECV = 0.1
# kick_power
KICK_POWER = 0.3
# dorrible_power
DRIBBLE_POWER = 0.6

# ボールがplacementされたとみなされる範囲
BALL_PLACE_TRESHOLD = 0.10
# 実際にボールがplacementされたとみなされる範囲
BALL_PLACE_AREA = 0.15
# ボールを置きに行く動作に入るときの範囲(ドリブルしない）
BALL_PLACE_AREA_NO_DRIBBLE = 0.3
# ボールを置きに行く動作に入るときの範囲
# BALL_PLACE_AREA = 0.5
# ボールに近いと判断する距離
BALL_GET_AREA = 4.0
# ボールが動いていると判断する速度
VEL_THRESHOLD = 0.5

# ボール保持の判定
IS_LOOK_TARGET_ANGLE = 4  # deg
IS_TOUCH_DIST = 0.20

# 侵入禁止をする範囲(余裕を見て+0.1)
BALL_MARGIN_DIST = 0.5 + 0.15

# 指定位置に到達したか判定
def threshold_pose(tr_my_pose):

    flag = False
    if tr_my_pose.x < 0.05 and math.fabs(tr_my_pose.y) < 0.05 and \
            math.fabs(tr_my_pose.theta) < math.radians(IS_LOOK_TARGET_ANGLE): 
        flag = True

    return flag

# 指定距離以内に到達したか判定
def threshold_dist(dist):

    flag = False
    if dist < 0.1:
        flag = True
    
    return flag

# 目標座標の後ろに座標を生成する
def generate_target_back_pose(trans, target_pose, margin_value):

    tr_target_back_pose = trans.transform(target_pose)
    tr_target_back_pose.x = tr_target_back_pose.x + margin_value
    tr_target_back_pose.y = tr_target_back_pose.y
    target_back_pose = trans.inverted_transform(tr_target_back_pose)
    
    return target_back_pose

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

# ball placementrを行う
# atkとrecvの2台で行う
def basic_ball_placement(control_target, target_pose, ball_info, robot_info, my_id, mode='atk'):
 
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
        control_target, avoid_ball = basic_atk(control_target, ball_info, atk_pose, recv_pose, target_pose)
    else:
        control_target, avoid_ball = basic_recv(control_target, ball_info, atk_pose, recv_pose, target_pose)
    
    return control_target, avoid_ball

# ball_placement(atkロボット)の動作
def basic_atk(control_target, ball_info, atk_pose, recv_pose, target_pose):

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, target_pose)

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)

    # 座標変換
    tr_atk_pose = trans.transform(atk_pose)

    # ボールとpalcement位置の距離
    dist_ball_to_target = tool.distance_2_poses(ball_info.pose, target_pose)

    # レシーブを行う座標を生成
    target_back_pose = generate_target_back_pose(trans, target_pose, SET_POSE_ADD_X_RECV)

    # recvとレシーブを行う座標との距離
    dist_recv_to_target_back = tool.distance_2_poses(recv_pose, target_back_pose)

    # ボール速度
    ball_vel = ball_info.velocity
    v = math.hypot(ball_vel.x, ball_vel.y)

    # placementの行動生成
    new_pose = Pose2D()
    avoid_ball = False

    # ボールが範囲に入っていない場合は処理を行う
    if BALL_PLACE_TRESHOLD < dist_ball_to_target:

        # 指定位置に到着したかどうか
        atk_flag = threshold_pose(tr_atk_pose)
        recv_flag = threshold_dist(dist_recv_to_target_back)
        
        # ---------------------------------------
        # 蹴ったあとにatkが追いかけない様に対策
        if VEL_THRESHOLD < v:
            new_pose = atk_pose
            control_target.dribble_power = 0

        elif (tr_atk_pose.y < -0.05 or 0.05 < tr_atk_pose.y) and 0.1 < dist_recv_to_target_back:
            new_pose = trans.inverted_transform(Pose2D(-SET_POSE_ADD_X, 0, 0))
            new_pose.theta = angle_ball_to_target
            
            # ドリブルする
            control_target.kick_power = 0.0
            control_target.dribble_power = DRIBBLE_POWER

            avoid_ball = True

        # もしボールとゴールに近い場合はアタッカーが置きにいく
        elif dist_ball_to_target < BALL_PLACE_AREA_NO_DRIBBLE:
            # ドリブルする
            control_target = offense.inplay_dribble(atk_pose, ball_info, 
                    control_target, target_pose)
            new_pose = control_target.path[-1]
            control_target.dribble_power = 0.0

        # もしボールとゴールに近い場合はアタッカーが置きにいく
        # ボールを拾いに行く
        elif dist_ball_to_target < BALL_GET_AREA:
            
            # ドリブルする
            control_target = offense.inplay_dribble(atk_pose, ball_info, 
                    control_target,target_pose)
            new_pose = control_target.path[-1]
            control_target.kick_power = 0.0
            control_target.dribble_power = DRIBBLE_POWER

        # お互いの位置がセットされたら蹴る
        elif atk_flag and recv_flag:

            # ボールを確実に保持するためボールの少し前に移動する
            new_pose = trans.inverted_transform(Pose2D(0.2, 0, 0))
            new_pose.theta = angle_ball_to_target

            # ドリブルとキックをオン
            control_target.kick_power = KICK_POWER
            control_target.dribble_power = DRIBBLE_POWER

        # ボールの後ろに移動
        else:
            # ボールの後ろに周り込む
            new_pose = trans.inverted_transform(Pose2D(-SET_POSE_ADD_X, 0, 0))
            new_pose.theta = angle_ball_to_target

            # ドリブルとキックをオフ
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0

            avoid_ball = True

    # 範囲内にボールがある場合は離れる
    else:
        # ボールから離れる動作
        angle_atk_to_target = tool.get_angle(target_pose, atk_pose)
        new_pose.x = target_pose.x + BALL_MARGIN_DIST * math.cos(angle_atk_to_target)
        new_pose.y = target_pose.y + BALL_MARGIN_DIST * math.sin(angle_atk_to_target)
        new_pose.theta = angle_atk_to_target + math.pi

        # 引くだけなのでドリブラを切る
        control_target.dribble_power = 0.0
        control_target.kick_power = 0.0

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

    # レシーブを行う座標を生成
    target_back_pose = generate_target_back_pose(trans, target_pose, SET_POSE_ADD_X_RECV)

    # 座標変換
    tr_target_back_pose = trans.transform(target_back_pose)

    # ボールの速度
    ball_vel = ball_info.velocity
    v = math.hypot(ball_vel.x, ball_vel.y)

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, target_pose)

    new_pose = Pose2D()
    avoid_ball = True
    if BALL_PLACE_TRESHOLD < dist_ball_to_target:

        # 速度が出ている場合レシーブしにいく
        if VEL_THRESHOLD < v:
            
            # レシーブするための座標生成
            new_pose = receive_ball(ball_info, recv_pose)

            # ドリブルする
            control_target.kick_power = 0
            control_target.dribble_power = DRIBBLE_POWER

        # placement座標の少し後ろに移動する
        else:
            new_pose = trans.inverted_transform(tr_target_back_pose)
            new_pose.theta = angle_ball_to_target + math.pi

            # ドリブルとキックをオフ
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0
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
def avoid_ball_place_line(my_pose, ball_info, placement_pose, control_target, force_avoid=False):

    angle_ball2placement = tool.get_angle(ball_info.pose, placement_pose)

    trans = tool.Trans(ball_info.pose, angle_ball2placement)
    tr_my_pose = trans.transform(my_pose)
    tr_placement_pose = trans.transform(placement_pose)
    dist_ball2placement = tool.distance_2_poses(ball_info.pose, placement_pose)

    # ライン上にいるやつは避ける
    # if -BALL_MARGIN_DIST < tr_my_pose.x < tr_placement_pose.x + BALL_MARGIN_DIST and \
        # -BALL_MARGIN_DIST < tr_my_pose.y < BALL_MARGIN_DIST:
    if BALL_PLACE_AREA_NO_DRIBBLE < dist_ball2placement or force_avoid:
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
    ball_vel = ball_info.velocity
    _can_receive_dist = 1.0
    _can_receive_hysteresis = 0.3

    target_pose = Pose2D()

    if Observer.ball_is_moving():
        angle_velocity = tool.get_angle_from_center(ball_vel)
        trans = tool.Trans(ball_pose, angle_velocity)

        tr_pose = trans.transform(my_pose)

        fabs_y = math.fabs(tr_pose.y)

        tr_pose.y = 0.0
        inv_pose = trans.inverted_transform(tr_pose)
        angle_to_ball = tool.get_angle(inv_pose, ball_pose)
        target_pose = Pose2D(inv_pose.x, inv_pose.y, angle_to_ball)

    return target_pose
