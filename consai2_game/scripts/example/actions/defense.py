# coding: UTF-8

# defense.pyでは、ボールを蹴らないactionを定義する

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


def defence_goal(my_pose, ball_info, control_target, my_role, defence_num):
    MARGIN_LINE = 0.1
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
            if my_pose.y < left_penalty_corner.y:
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
            if my_pose.y > right_penalty_corner.y:
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

    return target_pose
    

    


    
    

