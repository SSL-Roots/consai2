# coding: UTF-8
# defense.pyでは、ボールを蹴らないactionを定義する

import math
import rospy
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
sys.path.append(os.pardir)
from field import Field
from observer import Observer
import role
import tool


# defenseのroleによって行動を切り替える
def defense_decision(my_role, ball_info, control_target, my_pose, defense_num, robot_info, zone_enable=False):
    # ゴール前ディフェンス
    if role.ROLE_ID['ROLE_DEFENSE_GOAL_1'] <= my_role <= role.ROLE_ID['ROLE_DEFENSE_GOAL_2']:
        return defense_goal(my_pose, ball_info, control_target, my_role, defense_num)
    # ゾーンディフェンス
    elif role.ROLE_ID['ROLE_DEFENSE_ZONE_1'] <= my_role <= role.ROLE_ID['ROLE_DEFENSE_ZONE_4']:
        return defense_zone(my_pose, ball_info, control_target, my_role, defense_num, robot_info['their'], zone_enable)
    # 例外だった場合はその場にいる
    else:
        control_target.path = []
        control_target.path.append(my_pose)
        return control_target


# ゴール前ディフェンス
def defense_goal(my_pose, ball_info, control_target, my_role, defense_num):
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
        if my_role == role.ROLE_ID["ROLE_DEFENSE_GOAL_1"]:
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
    

# ゾーンディフェンス
def defense_zone(my_pose, ball_info, control_target, my_role, defense_num, their_robot_info, zone_enable):
    # ゴールディフェンスに割り当てる台数
    GOAL_DEFENSE_NUM = 2
    # 現在のディフェンス数 - ゴールディフェンス数 = ゾーンディフェンスに割り当てられる台数
    ZONE_DEFENSE_NUM = defense_num - GOAL_DEFENSE_NUM
    # ゾーンディフェンスが始まるROLE_ID
    ZONE_START_ROLE_ID = role.ROLE_ID["ROLE_DEFENSE_ZONE_1"]
    # ゾーンオフェンス用の待機場所
    ZONE_OFFENCE_POSE = Pose2D(3,0,0)

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

    # ゾーンオフェンス判定用フラグ
    my_role_is_offence = False

    # ボールが相手フィールドにあるとき
    # ゾーンから1台ゾーンオフェンスに出す
    # 相手キックオフ時などに前に出ないように
    # マージンを持って相手フィールド側かを判断している
    if ZONE_DEFENSE_NUM > 1 and ball_pose.x > MARGIN_CENTER:
        # 1台ディフェンスが減る
        ZONE_DEFENSE_NUM -= 1
        # ゾーンディフェンスが始まるROLE_IDをずらす
        ZONE_START_ROLE_ID = role.ROLE_ID["ROLE_DEFENSE_ZONE_2"]
        # ROLE_DEFENSE_ZONE_1をゾーンオフェンスとして出す
        if my_role is role.ROLE_ID["ROLE_DEFENSE_ZONE_1"]:
            my_role_is_offence = True

    # 私はゾーンオフェンスです
    if my_role_is_offence:
        zone_id = 0
        target_pose = ZONE_OFFENCE_POSE
        # 基本的にアタッカーがボールを取りに行くので
        # ボールが無い方向に移動してこぼれ球が取れるようにする
        if ball_pose.y > 0:
            target_pose.y  =  - quarter_field_width
        else:
            target_pose.y = quarter_field_width
        # ボールを向く
        target_pose.theta = angle_to_ball

    # ゾーンオフェンス以外
    if ZONE_DEFENSE_NUM > 0 and not my_role_is_offence:
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
            invader_pose = [i.pose for i in their_robot_info \
                    if split_field[zone_id * 2] < i.pose.y < split_field[(zone_id + 1) * 2] and \
                    i.pose.x < 0]

            # ボールが自分のゾーンの中に入っている, かつzone_enable
            if(zone_enable and \
                    ball_pose.x < 0 and \
                    split_field[zone_id * 2] < ball_pose.y < split_field[(zone_id + 1) * 2]):
                trans = tool.Trans(ball_pose, angle_to_ball_from_goal)
                target_pose = trans.inverted_transform(Pose2D(-0.9, 0, 0))
            # 自分のゾーンにボールはないけど敵がいる場合は割り込む
            elif zone_enable and invader_pose != []:
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
        receive_ball_result, receive_target_pose = update_receive_ball(ball_info, my_pose, zone_id)
        if receive_ball_result:
            # ドリブラー回す
            control_target.dribble_power = DRIBBLE_POWER
            target_pose = receive_target_pose

    # ペナルティエリアには入らない
    if((left_penalty_corner.y + 0.2 > target_pose.y > right_penalty_corner.y - 0.2) and \
                target_pose.x < left_penalty_corner.x + 0.3):
        target_pose.x = half_our_field_length

    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

class Receiving(object):
    _recenving = [False] * role.ZONE_DEFENSE_NUM

    @classmethod
    def update_receiving(cls, zone_id, param):
        Receiving._recenving[zone_id] = param
    @classmethod
    def receiving(cls, zone_id):
        return Receiving._recenving[zone_id]


def update_receive_ball(ball_info, my_pose, zone_id):
    ball_pose = ball_info.pose
    ball_vel = ball_info.velocity
    _can_receive_dist = 1.0
    _can_receive_hysteresis = 0.3

    result = False

    target_pose = Pose2D()

    if Observer.ball_is_moving():
        angle_velocity = tool.get_angle_from_center(ball_vel)
        trans = tool.Trans(ball_pose, angle_velocity)

        tr_pose = trans.transform(my_pose)

        fabs_y = math.fabs(tr_pose.y)

        if Receiving.receiving(zone_id) == False and \
                fabs_y < _can_receive_dist - _can_receive_hysteresis:
            Receiving.update_receiving(zone_id, True)

        elif Receiving.receiving(zone_id) == True and \
                fabs_y > _can_receive_dist + _can_receive_hysteresis:
            Receiving.update_receiving(zone_id, False)

        if Receiving.receiving(zone_id) and tr_pose.x > 0.0:

            tr_pose.y = 0.0
            inv_pose = trans.inverted_transform(tr_pose)
            angle_to_ball = tool.get_angle(inv_pose, ball_pose)
            target_pose = Pose2D(inv_pose.x, inv_pose.y, angle_to_ball)
            result = True

    return result, target_pose

