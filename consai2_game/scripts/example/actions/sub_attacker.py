# coding: UTF-8
# defense.pyでは、ボールを蹴らないactionを定義する

import math
import rospy
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo, ControlTarget
from geometry_msgs.msg import Pose2D

sys.path.append(os.pardir)
from field import Field
import role
import tool
import defense


# サブアタッカー
def sub_attacker(my_pose, ball_info, control_target, my_role, defense_num, their_robot_info, role_action_enable):
    # ゾーンオフェンス用の待機場所
    ZONE_OFFENCE_POSE = Pose2D(Field.field('length')*0.25 ,0,0)

    # センターライン用のマージン
    MARGIN_CENTER = 0.6
    # ちょっと前進用のマージン
    MARGIN_LITTLE_FORWARD = 1.0

    # ドリブルパワー
    DRIBBLE_POWER = 0.6

    # ボール位置
    ball_pose = ball_info.pose

    # Field情報からペナルティエリアの情報を取得
    # フィールド幅
    field_width = Field.field('width')
    # フィールド幅の半分
    half_field_width = float(field_width) / 2
    # フィールド幅の1/4
    quarter_field_width = float(field_width) / 4
    # フィールド長さ
    field_length = Field.field('length')
    # フィールド長さの1/4 → 自陣側の長さの半分
    half_our_field_length = -float(field_length) / 4
    # ゴール中心
    goal_center = Field.goal_pose('our', 'center')

    # ペナルティエリアの角
    left_penalty_corner = Field.penalty_pose('our', 'upper_front')
    right_penalty_corner = Field.penalty_pose('our', 'lower_front')

    # 自分からボールへの角度（ボールの方向を向くため）
    angle_to_ball = tool.get_angle(my_pose, ball_pose)
    # ゴール中心からボールへの角度
    angle_to_ball_from_goal = tool.get_angle(goal_center, ball_pose)

    # ゾーンディフェンス用のID
    zone_id = 0
    # 移動目標点の初期化
    target_pose = Pose2D()

    # ---------------------------------------------------------
    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    # ゾーンオフェンス判定用フラグ
    my_role_is_offence = False

    # 私はゾーンオフェンスです
    target_pose = ZONE_OFFENCE_POSE
    if role_action_enable:
        # 基本的にアタッカーがボールを取りに行くので
        # ボールが無い方向に移動してこぼれ球が取れるようにする
        if ball_pose.y > 0:
            target_pose.y  =  - quarter_field_width
        else:
            target_pose.y = quarter_field_width
    else:
        target_pose.x = -1.5
        target_pose.y = 0

    # ボールを向く
    target_pose.theta = angle_to_ball

    # ボールが来てたらボールを受け取る
    # TODO: update_receive_ball
    receive_ball_result, receive_target_pose = defense.update_receive_ball(ball_info, my_pose, 0)
    if receive_ball_result:
        # ドリブラー回す
        control_target.dribble_power = DRIBBLE_POWER
        target_pose = receive_target_pose
    else:
        # ボールに近づいてたら離れる
        target_pose = defense.get_avoid_ball_pose(ball_pose, target_pose)

    # ペナルティエリアには入らない
    if((left_penalty_corner.y + 0.2 > target_pose.y > right_penalty_corner.y - 0.2) and \
                target_pose.x < left_penalty_corner.x + 0.3):
        target_pose.x = half_our_field_length

    control_target.path = []
    control_target.path.append(target_pose)

    return control_target
