# coding: UTF-8

# defense.pyでは、ボールを蹴らないactionを定義する

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool
from observer import Observer

sys.path.append(os.pardir)
from field import Field
import role

def interpose(target_info, control_target, 
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
    # remake_path is Trueならpathを再生成する
    # pathを再生成すると衝突回避用に作られた経路もリセットされる
    if remake_path:
        control_target.path = []
        control_target.path.append(new_goal_pose)

    return control_target

def defence_decision(my_role, ball_info, control_target, my_pose, defence_num, robot_info, zone_enable=False):
    if role.ROLE_ID['ROLE_DEFENCE_GOAL_1'] <= my_role <= role.ROLE_ID['ROLE_DEFENCE_GOAL_2']:
        return defence_goal(my_pose, ball_info, control_target, my_role, defence_num)
    elif role.ROLE_ID['ROLE_DEFENCE_ZONE_1'] <= my_role <= role.ROLE_ID['ROLE_DEFENCE_ZONE_4']:
        return defence_zone(my_pose, ball_info, control_target, my_role, defence_num, robot_info['their'], zone_enable)
    else:
        control_target.path = []
        control_target.path.append(my_pose)
        return control_target


# ゴール前ディフェンス
def defence_goal(my_pose, ball_info, control_target, my_role, defence_num):
    MARGIN_LINE = 0.2
    MARGIN_ROBOT = 0
    MARGIN_FOR_SPEED = 0.5
    if defence_num > 1:
        if my_role == role.ROLE_ID["ROLE_DEFENCE_GOAL_1"]:
            MARGIN_ROBOT = 0.15
        else:
            MARGIN_ROBOT = -0.15

    ball_pose = ball_info.pose
    
    ball_is_center = False
    ball_is_left = False
    ball_is_right = False
    my_pose_is_left = False
    my_pose_is_right = False
    target_is_center = False
    target_is_left = False
    target_is_right = False

    # 到達姿勢の計算とcontrol_targetの更新(path以外)
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    left_penalty_corner = Field.penalty_pose('our', 'upper_front')
    right_penalty_corner = Field.penalty_pose('our', 'lower_front')
    left_penalty_goalside = Field.penalty_pose('our', 'upper_back')
    right_penalty_goalside = Field.penalty_pose('our', 'lower_back')
    goal_center = Field.goal_pose('our', 'center')

    angle_to_left_penalty_corner =  tool.get_angle(goal_center, left_penalty_corner)
    angle_to_right_penalty_corner = tool.get_angle(goal_center, right_penalty_corner)
    angle_to_ball = tool.get_angle(my_pose, ball_pose)
    
    # ゴールを背にした左コーナー中心の座標軸
    trans_left = tool.Trans(left_penalty_corner, angle_to_left_penalty_corner)
    tr_left_ball_pose = trans_left.transform(ball_pose)

    # ゴールを背にした右コーナー中心の座標軸
    trans_right = tool.Trans(right_penalty_corner, angle_to_right_penalty_corner)
    tr_right_ball_pose = trans_right.transform(ball_pose)

    # ボールの位置を判定
    if tr_left_ball_pose.y > 0:
        ball_is_left = True
    elif tr_right_ball_pose.y < 0:
        ball_is_right = True
    else:
        ball_is_center = True

    # ボールは真ん中にある
    if ball_is_center:
        target_pose = tool.get_intersection(left_penalty_corner, right_penalty_corner,
                goal_center, ball_pose)
        if target_pose is not None:
            target_pose.x += MARGIN_LINE
            # ロボットが後ろにいる
            if my_pose.x < left_penalty_corner.x:
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
        target_pose = tool.get_intersection(left_penalty_corner, left_penalty_goalside,
                goal_center, ball_pose)
        if target_pose is not None:
            target_pose.y += MARGIN_LINE
            # ロボットが左側にいない
            if my_pose.y < left_penalty_corner.y - 0.5:
                # 左側にいないかつ後ろにいる場合は右側を沿う
                if my_pose.x < left_penalty_corner.x:
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
            target_pose.y -= MARGIN_LINE
            # ロボットが右側にいない
            if my_pose.y > right_penalty_corner.y + 0.5:
                # 右側にいないかつ後ろにいる場合は左側を沿う
                if my_pose.x < left_penalty_corner.x:
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
def defence_zone(my_pose, ball_info, control_target, my_role, defence_num, their_robot_info, zone_enable):
    ROLE_MAX = 7
    GOAL_DEFENCE_NUM = 2
    ZONE_DEFENCE_NUM = defence_num - GOAL_DEFENCE_NUM

    ball_pose = ball_info.pose

    # control_targetの更新(path以外)
    control_target.kick_power = 0.0
    control_target.dribble_power = 0.0

    field_width = Field.field('width')
    half_field_width = float(field_width) / 2
    field_length = Field.field('length')
    half_our_field_length = -float(field_length) / 4
    goal_center = Field.goal_pose('our', 'center')

    # ペナルティエリアの角
    left_penalty_corner = Field.penalty_pose('our', 'upper_front')
    right_penalty_corner = Field.penalty_pose('our', 'lower_front')

    angle_to_ball = tool.get_angle(my_pose, ball_pose)
    angle_to_ball_from_goal = tool.get_angle(goal_center, ball_pose)

    zone_id = None
    target_pose = Pose2D()

    if ZONE_DEFENCE_NUM > 0:
        step = float(field_width) / (ZONE_DEFENCE_NUM * 2)
        split_field = [i * step - half_field_width for i in range(0,(ZONE_DEFENCE_NUM * 2 + 1))]
        # 今のディフェンス数からゾーンの区切りを変える
        split_field_center = [i * step - half_field_width for i in range(0,(ZONE_DEFENCE_NUM * 2)) \
                if i % 2 != 0]

        # 参照エラー対策
        try:
            zone_id = my_role - role.ROLE_ID["ROLE_DEFENCE_ZONE_1"]
            target_pose.y = split_field_center[zone_id]
            # 自分のゾーンに入っている敵チェック
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
                target_pose.x = half_our_field_length
        except IndexError:
            target_pose = my_pose
        target_pose.theta = angle_to_ball


    if zone_id != None:
        receive_ball_result, receive_target_pose = update_receive_ball(ball_info, my_pose, zone_id)
        if receive_ball_result:
            # ドリブラー回す
            control_target.dribble_power = 1.0
            target_pose = receive_target_pose

    # ペナルティエリアには入らない
    if((left_penalty_corner.y + 0.2 > target_pose.y > right_penalty_corner.y - 0.2) and \
                target_pose.x < left_penalty_corner.x + 0.3):
        target_pose.x = half_our_field_length

    control_target.path = []
    control_target.path.append(target_pose)

    return control_target

class Receiving(object):
    _recenving = [False] * role.ZONE_DEFENCE_NUM

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

