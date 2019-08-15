#D6 coding: UTF-8

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool

sys.path.append(os.pardir)
from field import Field


def atk(my_pose, ball_info, control_target, goal_pose, your_id, robot_info):

    KICK_POWER = 0.5
    DRIBBLE_POWER = 0.5
    
     # パス受取ロボットの座標
    dist = []
    for our_info in robot_info['our']:
        dist.append(tool.distance_2_poses(our_info.pose, goal_pose)) 
    your_id = dist.index(min(dist))
    your_pose = robot_info['our'][your_id].pose
    dist_your2goal = min(dist)

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, goal_pose)

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_my_pose = trans.transform(my_pose)
    tr_your_pose = trans.transform(your_pose)
    tr_goal_pose = trans.transform(goal_pose)
    tr_ball_pose = trans.transform(ball_info.pose)
    
    tr_goal_back_pose = tr_goal_pose
    tr_goal_back_pose.x += 0.3

    # レシーブと対象の位置の距離 
    dist_your2ball = tool.distance_2_poses(tr_your_pose, tr_ball_pose)
    dist_your2goal = tool.distance_2_poses(tr_your_pose, tr_goal_pose)
    # 自分と対象の位置の距離 
    dist_i2ball = tool.distance_2_poses(tr_my_pose, tr_ball_pose)
    dist_i2goal = tool.distance_2_poses(tr_my_pose, tr_goal_pose)

    dist_ball2goal = tool.distance_2_poses(ball_info.pose, goal_pose)
    if 0.15 < dist_ball2goal:

        # ロボットがボールの裏側に回ったらcan_kick is True
        my_flag = False
        if tr_my_pose.x < 0.05 and math.fabs(tr_my_pose.y) < 0.05:
            my_flag = True

        your_flag = False
        if dist_your2goal < 0.05:

            your_flag = True
        # if tr_your_pose.x < 0.05 and math.fabs(tr_your_pose.y) < 0.05:
            # your_flag = True
        
        avoid_ball = True # ボールを回避する
        new_goal_pose = Pose2D()
        if my_flag and your_flag:
            # ボールをける
            avoid_ball = False

            # ボールの前方に移動する
            new_position = trans.inverted_transform(Pose2D(0.1, 0, 0))
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target
            # ドリブルとキックをオン
            control_target.kick_power = KICK_POWER
            control_target.dribble_power = DRIBBLE_POWER

        # ボールを置きにいく
        elif dist_i2goal < 0.5:
            avoid_ball = False

            # ボールの前方に移動する
            new_position = trans.inverted_transform(tr_goal_pose)
            new_goal_pose = new_position
            new_goal_pose.theta = tool.get_angle(my_pose, ball_info.pose)

            control_target.kick_power = 0
            control_target.dribble_power = DRIBBLE_POWER
            if dist_i2goal < 0.1:
                control_target.dribble_power = 0

        else:
            # ボールの裏に移動する
            new_position = trans.inverted_transform(Pose2D(-0.3, 0, 0))
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target
            # ドリブルとキックをオフ
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0
    else:
        avoid_ball = False
        new_goal_pose = my_pose

    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target, avoid_ball

# ボールを受け取る側の動作
def recv(my_pose, ball_info, control_target, goal_pose, your_id, robot_info, can_kick=False):

    DRIBBLE_POWER = 0.8

    # ペアになるロボットの座標
    your_pose = robot_info['our'][your_id].pose
    
    dist_i2ball = tool.distance_2_poses(my_pose, goal_pose)

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, goal_pose)

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_my_pose = trans.transform(my_pose)
    tr_goal_pose = trans.transform(goal_pose)

    # もしボールが範囲内なら以降無視
    dist_ball2goal = tool.distance_2_poses(ball_info.pose, goal_pose)
    if 0.15 < dist_ball2goal:

        # ロボットがボールの裏側に回ったらcan_kick is True
        can_kick = False
        if tr_my_pose.x < 0.05 and math.fabs(tr_my_pose.y) < 0.05:
            can_kick = True
        
        avoid_ball = True # ボールを回避する
        new_goal_pose = Pose2D()
        if can_kick and dist_i2ball < 0.3:

            # ボールをける
            avoid_ball = False

            # ゴールに移動する
            new_position = trans.inverted_transform(tr_goal_pose)
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target + math.pi
            # ドリブルとキックをオン
            control_target.dribble_power = DRIBBLE_POWER
        else:
            # ボールの裏に移動する
            tr_target_pose = tr_goal_pose
            tr_target_pose.x += 0.3
            new_position = trans.inverted_transform(tr_target_pose)
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target + math.pi
            # ドリブルとキックをオフ
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0
    else:
        new_goal_pose = my_pose
        avoid_ball = False
            
    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target, avoid_ball


