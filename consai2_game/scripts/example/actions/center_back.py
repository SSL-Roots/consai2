# coding: UTF-8

import math
import rospy
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo, ControlTarget
from geometry_msgs.msg import Pose2D

sys.path.append(os.pardir)
from field import Field
import role
import tool


# ゴール前ディフェンス
def center_back(my_pose, ball_info, control_target, my_role, defense_num):
    # ゴール前ディフェンスは、ペナルティエリアに沿って守備を行う
    # ボールとゴールを結ぶ直線と、ペナルティエリアのラインの交点を基準に移動
    # ボールの位置によって、ライン左、ライン正面、ライン右のどのラインの前に移動するか変わる

    # ペナルティエリアからどれだけ離れるか
    MARGIN_LINE = 0.2
    # 2台でディフェンスする時のお互いの距離
    MARGIN_ROBOT = 0
    # ライン左からライン正面に移動する時等に、
    # 一時的にスピードアップするための数値
    MARGIN_FOR_SPEED = 0.5

    # ディフェンスが2台以上いる時はMARGIN_ROBOTを変更
    if defense_num > 1:
        if my_role == role.ROLE_ID["ROLE_CENTER_BACK_1"]:
            MARGIN_ROBOT = 0.15
        else:
            MARGIN_ROBOT = -0.15

    # ボール位置
    ball_pose = ball_info.pose

    # ボールの位置判定用フラグ
    ball_is_center = False
    ball_is_left = False
    ball_is_right = False
    # 自分の位置判定用フラグ
    my_pose_is_left = False
    my_pose_is_right = False
    # 移動すべき場所判定用フラグ
    target_is_center = False
    target_is_left = False
    target_is_right = False

    # Field情報からペナルティエリアの情報を取得
    # ペナルティエリアの左角
    left_penalty_corner = Field.penalty_pose('our', 'upper_front')
    # ペナルティエリアの右角
    right_penalty_corner = Field.penalty_pose('our', 'lower_front')
    # ペナルティエリアの左側のゴールラインとの交点
    left_penalty_goalside = Field.penalty_pose('our', 'upper_back')
    # ペナルティエリアの右側のゴールラインとの交点
    right_penalty_goalside = Field.penalty_pose('our', 'lower_back')
    # ゴールの中心
    goal_center = Field.goal_pose('our', 'center')

    # ゴール中心からペナルティエリア左角への角度
    angle_to_left_penalty_corner =  tool.get_angle(goal_center, left_penalty_corner)
    # ゴール中心からペナルティエリア右角への角度
    angle_to_right_penalty_corner = tool.get_angle(goal_center, right_penalty_corner)
    # 自分からボールへの角度（ボールの方向を向くため）
    angle_to_ball = tool.get_angle(my_pose, ball_pose)
    
    # ゴールを背にした左角を中心とした座標軸へ変換
    trans_left = tool.Trans(left_penalty_corner, angle_to_left_penalty_corner)
    tr_left_ball_pose = trans_left.transform(ball_pose)

    # ゴールを背にした右角を中心とした座標軸へ変換
    trans_right = tool.Trans(right_penalty_corner, angle_to_right_penalty_corner)
    tr_right_ball_pose = trans_right.transform(ball_pose)

    # ボールの位置を判定
    if tr_left_ball_pose.y > 0:
        ball_is_left = True
    elif tr_right_ball_pose.y < 0:
        ball_is_right = True
    else:
        ball_is_center = True

    # ---------------------------------------------------------
    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    # ボールは真ん中にある
    if ball_is_center:
        # ペナルティエリアの左角と右角の線分(正面の線)と、ゴール中心とボールの線分の交点
        target_pose = tool.get_intersection(left_penalty_corner, right_penalty_corner,
                goal_center, ball_pose)
        if target_pose is not None:
            # ペナルティエリアに侵入しないように+MARGIN_LINE
            target_pose.x += MARGIN_LINE
            # ロボットが正面の線より後ろにいる
            if my_pose.x < left_penalty_corner.x:
                # ダッシュで正面に移動
                target_pose.x += MARGIN_FOR_SPEED
                # ペナルティエリアを沿って移動
                if my_pose.y > 0:
                    target_pose.y = left_penalty_corner.y + MARGIN_LINE
                else:
                    target_pose.y = right_penalty_corner.y - MARGIN_LINE
            else:
                target_pose.y += MARGIN_ROBOT
        else:
            target_pose = Pose2D()
    # ボールは左側にある
    elif ball_is_left:
        # ペナルティエリアの左側の線分と、ゴール中心とボールの線分の交点
        target_pose = tool.get_intersection(left_penalty_corner, left_penalty_goalside,
                goal_center, ball_pose)
        if target_pose is not None:
            # ペナルティエリアに侵入しないように+MARGIN_LINE
            target_pose.y += MARGIN_LINE
            # ロボットが左側にいない
            if my_pose.y < left_penalty_corner.y:
                # 左側にいないかつ後ろにいる場合は右側を沿う
                if my_pose.x < left_penalty_corner.x and my_pose.y < 0:
                    target_pose.x = left_penalty_corner.x + MARGIN_FOR_SPEED
                    target_pose.y = right_penalty_corner.y - MARGIN_LINE
                # 左側にダッシュで移動
                else:
                    target_pose.x = left_penalty_corner.x + MARGIN_LINE
                    target_pose.y += MARGIN_FOR_SPEED
            else:
                target_pose.x -= MARGIN_ROBOT
        else:
            target_pose = Pose2D()
    # ボールは右側にある
    elif ball_is_right:
        target_pose = tool.get_intersection(right_penalty_corner, right_penalty_goalside,
                goal_center, ball_pose)
        if target_pose is not None:
            # ペナルティエリアに侵入しないように-MARGIN_LINE
            target_pose.y -= MARGIN_LINE
            # ロボットが右側にいない
            if my_pose.y > right_penalty_corner.y:
                # 右側にいないかつ後ろにいる場合は左側を沿う
                if my_pose.x < left_penalty_corner.x and my_pose.y > 0:
                    target_pose.x = left_penalty_corner.x + MARGIN_FOR_SPEED
                    target_pose.y = left_penalty_corner.y + MARGIN_LINE
                # 右側にダッシュで移動
                else:
                    target_pose.x = right_penalty_corner.x + MARGIN_LINE
                    target_pose.y -= MARGIN_FOR_SPEED
            else:
                target_pose.x += MARGIN_ROBOT
        else:
            target_pose = Pose2D()
    # フィールドから出ないように
    if target_pose.x < goal_center.x:
        target_pose.x = goal_center.x
    # 向きはボールの方向
    target_pose.theta = angle_to_ball

    control_target.path = []
    control_target.path.append(target_pose)

    return control_target
