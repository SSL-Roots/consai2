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
import defense


# ゾーンディフェンス
def defense_zone(my_pose, ball_info, control_target, my_role, defense_num, their_robot_info, role_action_enable):
    # ゴールディフェンスに割り当てる台数
    GOAL_DEFENSE_NUM = 2
    # 現在のディフェンス数 - ゴールディフェンス数 = ゾーンディフェンスに割り当てられる台数
    ZONE_DEFENSE_NUM = defense_num - GOAL_DEFENSE_NUM
    # ゾーンディフェンスが始まるROLE_ID
    ZONE_START_ROLE_ID = role.ROLE_ID["ROLE_ZONE_1"]
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
    zone_id = None
    # 移動目標点の初期化
    target_pose = Pose2D()

    # ---------------------------------------------------------
    # キックとドリブルはOFF
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0


    # ゾーンオフェンス以外
    if ZONE_DEFENSE_NUM > 0:
        step = float(field_width) / (ZONE_DEFENSE_NUM * 2)
        # ゾーンディフェンスの数でフィールド幅を等分した配列を作る
        split_field = [i * step - half_field_width for i in range(0,(ZONE_DEFENSE_NUM * 2 + 1))]
        # ゾーンディフェンスの数でフィールド幅を等分した時の
        # それぞれのゾーンの中心の配列を作る
        split_field_center = [i * step - half_field_width for i in range(0,(ZONE_DEFENSE_NUM * 2)) \
                if i % 2 != 0]

        # 参照エラー対策のtry
        try:
            # ゾーンIDの計算
            # ロボットが8台生きていてゾーンオフェンスがいなければ
            # ゾーンIDは0~3
            zone_id = my_role - ZONE_START_ROLE_ID
            # ゾーンの中心を目標位置とする
            target_pose.y = split_field_center[zone_id]

            # 自分のゾーンに入っている敵チェック
            # their_robot_infoのposeを抜き出してそれぞれチェック
            # 敵が自陣側にいる、かつ、自分のゾーンの幅の中にいる、を確認
            # 当てはまらない場合配列は空っぽ
            invader_pose = [i.pose for i in their_robot_info \
                    if split_field[zone_id * 2] < i.pose.y < split_field[(zone_id + 1) * 2] and \
                    i.pose.x < 0]

            # ボールが自分のゾーンの中に入っている, かつrole_action_enable
            if(role_action_enable and \
                    ball_pose.x < 0 and \
                    split_field[zone_id * 2] < ball_pose.y < split_field[(zone_id + 1) * 2]):
                trans = tool.Trans(ball_pose, angle_to_ball_from_goal)
                target_pose = trans.inverted_transform(Pose2D(-0.9, 0, 0))
            # 自分のゾーンにボールはないけど敵がいる場合は割り込む
            elif role_action_enable and invader_pose != []:
                # 敵とボールの間に割り込む
                angle_to_ball_from_invader = tool.get_angle(invader_pose[0], ball_pose)
                trans = tool.Trans(invader_pose[0], angle_to_ball_from_invader)
                target_pose = trans.inverted_transform(Pose2D(0.5, 0, 0))
            else:
                # ボールが敵陣の時はディフェンスちょっと前進
                if ball_pose.x > MARGIN_CENTER:
                    target_pose.x = half_our_field_length + MARGIN_LITTLE_FORWARD
                else:
                    target_pose.x = half_our_field_length

        except IndexError:
            target_pose = my_pose
        target_pose.theta = angle_to_ball

    # ボールが来てたらボールを受け取る
    if zone_id != None:
        receive_ball_result, receive_target_pose = defense.update_receive_ball(ball_info, my_pose, my_role)
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
