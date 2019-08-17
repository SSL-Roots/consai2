#D6 coding: UTF-8

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool
import normal

sys.path.append(os.pardir)
from field import Field

from observer import Observer

# 後ろに周り込むときの位置取り
SET_POSE_ADD_X = 0.3
# kick_power
KICK_POWER = 0.5
# dorrible_power
DRIBBLE_POWER = 0.8

# ボールがplacementされたとみなされる範囲
BALL_PLACE_THRESHOLD = 0.15
# ボールを置きに行く動作に入るときの範囲
BALL_PLACE_AREA = 0.5
# ボールに近いと判断する距離
BALL_GET_AREA = 0.5
# ボールが動いていると判断する速度
VEL_THRESHOLD = 0.5

# ボール保持の判定
IS_LOOK_TARGET_ANGLE = 5  # deg
IS_TOUCH_DIST = 0.20

# 侵入禁止をする範囲(余裕を見て+0.1)
BALL_MARGIN_DIST = 0.5 + 0.2

# 指定位置に到達したか判定
def threshold(tr_my_pose):

    flag = False
    if tr_my_pose.x < 0.05 and math.fabs(tr_my_pose.y) < 0.05 and \
            math.fabs(tr_my_pose.theta) < math.radians(IS_LOOK_TARGET_ANGLE): 
        flag = True

    return flag

def atk(my_pose, ball_info, control_target, goal_pose, your_id, robot_info):

    # パス受取ロボットの座標
    dist = []
    for our_info in robot_info['our']:
        if our_info.disappeared is False:
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
    
    tr_goal_back_pose = Pose2D()
    tr_goal_back_pose.x = tr_goal_pose.x + SET_POSE_ADD_X
    tr_goal_back_pose.y = tr_goal_pose.y
    goal_back_pose = trans.inverted_transform(tr_goal_back_pose)

    # レシーブと対象の位置の距離 
    dist_your2ball = tool.distance_2_poses(tr_your_pose, tr_ball_pose)
    dist_your2goal = tool.distance_2_poses(tr_your_pose, tr_goal_pose)
    dist_your2goal_back = tool.distance_2_poses(tr_your_pose, tr_goal_back_pose)


    # 自分と対象の位置の距離 
    dist_i2ball = tool.distance_2_poses(tr_my_pose, tr_ball_pose)
    dist_i2goal = tool.distance_2_poses(tr_my_pose, tr_goal_pose)
    dist_i2goal_back = tool.distance_2_poses(tr_my_pose, tr_goal_back_pose)

    dist_ball2goal = tool.distance_2_poses(ball_info.pose, goal_pose)

    ball_vel = ball_info.velocity
    v = math.hypot(ball_vel.x, ball_vel.y)

    # ボールが範囲に入っていない場合は処理を行う
    if BALL_PLACE_THRESHOLD < dist_ball2goal:

        my_flag = threshold(tr_my_pose)

        your_flag = False
        if dist_your2goal_back < 0.1:

            your_flag = True

        avoid_ball = True # ボールを回避する
        new_goal_pose = Pose2D()

        # 蹴ったあとに追いかけない様に対策
        if VEL_THRESHOLD < v:
            new_goal_pose = my_pose
            control_target.kick_power = 0
        # もしボールとゴールに近い場合はアタッカーが置きにいく
        elif dist_ball2goal < BALL_PLACE_AREA:
            control_target.kick_power = 0
            if my_flag:
                avoid_ball = False
                new_position = trans.inverted_transform(tr_goal_pose)
                new_goal_pose.theta = tool.get_angle(my_pose, ball_info.pose)
                control_target.dribble_power = DRIBBLE_POWER
            else:
                # ボールの後ろに周り込む
                tr_target_pose = tr_ball_pose
                tr_target_pose.x -= 0.3
                new_goal_pose = trans.inverted_transform(tr_target_pose)
                new_goal_pose.theta = angle_ball_to_target
                new_goal_pose.theta = tool.get_angle(my_pose, ball_info.pose)
                control_target.dribble_power = 0

        # お互いの位置がセットされたら蹴る
        elif my_flag and your_flag:
            # ボールをける
            avoid_ball = False

            new_position = trans.inverted_transform(Pose2D(0.2, 0, 0))
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target
            # ドリブルとキックをオン
            control_target.kick_power = KICK_POWER
            control_target.dribble_power = DRIBBLE_POWER
            control_target.dribble_power = DRIBBLE_POWER

        # ボールを置きにいく
        elif dist_i2goal < BALL_PLACE_AREA and dist_i2goal_back < BALL_PLACE_AREA:
            avoid_ball = False
            # ball近づく
            if IS_TOUCH_DIST < dist_i2ball:
                # レシーブしにいく
                target_pose = receive_ball(ball_info, my_pose)
                target_pose.y = my_pose.y
                new_goal_pose = target_pose
                control_target.kick_power = 0
                control_target.dribble_power = DRIBBLE_POWER
            else:
                # 移動する
                new_goal_pose = trans.inverted_transform(tr_goal_pose)
                new_goal_pose.theta = tool.get_angle(my_pose, ball_info.pose)

                control_target.kick_power = 0
                control_target.dribble_power = DRIBBLE_POWER
        else:
            # ボールの裏に移動する
            new_position = trans.inverted_transform(Pose2D(-SET_POSE_ADD_X, 0, 0))
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target
            # ドリブルとキックをオフ
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0
    else:
        avoid_ball = False
        # tr_target_pose = tr_ball_pose
        # tr_target_pose.x -= -BALL_MARGIN_DIST
        # new_goal_pose = trans.inverted_transform(tr_target_pose)
        new_goal_pose = Pose2D()
        angle_i2goal = tool.get_angle(goal_pose, my_pose)
        new_goal_pose.x = goal_pose.x + BALL_MARGIN_DIST * math.cos(angle_i2goal)
        new_goal_pose.y = goal_pose.y + BALL_MARGIN_DIST * math.sin(angle_i2goal)
        new_goal_pose.theta = angle_i2goal + math.pi

        control_target.dribble_power = 0
        control_target.kick_power = 0

    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target, avoid_ball

# ボールを受け取る側の動作
def recv(my_pose, ball_info, control_target, goal_pose, your_id, robot_info):

    # ペアになるロボットの座標
    your_pose = robot_info['our'][your_id].pose
    
    dist_i2ball = tool.distance_2_poses(my_pose, goal_pose)

    # 目標座標までの角度
    angle_ball_to_target = tool.get_angle(ball_info.pose, goal_pose)

    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_my_pose = trans.transform(my_pose)
    tr_goal_pose = trans.transform(goal_pose)
    tr_ball_pose = trans.transform(ball_info.pose)

    # もしボールが範囲内なら以降無視
    dist_ball2goal = tool.distance_2_poses(ball_info.pose, goal_pose)
    dist_your2goal = tool.distance_2_poses(your_pose, goal_pose)
    dist_your2ball = tool.distance_2_poses(your_pose, ball_info.pose)


    dist_ball2goal = tool.distance_2_poses(ball_info.pose, goal_pose)

    # ボールと目標の距離
    dist_ball2target = tool.distance_2_poses(ball_info.pose, goal_pose)


    if BALL_PLACE_THRESHOLD < dist_ball2goal:

        flag = threshold(tr_my_pose)
        ball_vel = ball_info.velocity
        v = math.hypot(ball_vel.x, ball_vel.y)

        avoid_ball = True # ボールを回避する
        new_goal_pose = Pose2D()
        
        if VEL_THRESHOLD < v:
            target_pose = receive_ball(ball_info, my_pose)
            new_goal_pose = target_pose
            control_target.kick_power = 0
            if 0.3 < dist_i2ball:
                control_target.dribble_power = 0
            else:
                control_target.dribble_power = DRIBBLE_POWER
        # elif dist_your2goal < 0.3:
            # new_goal_pose = my_pose
            # control_target.kick_power = 0
            # control_target.dribble_power = 0

        # アタッカーのほうが近い場合は避ける
        elif dist_ball2target < BALL_PLACE_AREA:
            
            # new_position = trans.inverted_transform(tr_goal_pose)
            # new_goal_pose.x += BALL_MARGIN_DIST
            # new_goal_pose.y += BALL_MARGIN_DIST
            new_position = my_pose
            avoid_ball = False
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0

        elif dist_ball2goal < BALL_PLACE_AREA:
            avoid_ball = False

            # ゴールに移動する
            new_position = trans.inverted_transform(tr_goal_pose)
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target + math.pi
            # ドリブルとキックをオン
            control_target.dribble_power = DRIBBLE_POWER
        else:
            # ゴールの裏に移動する
            tr_target_pose = tr_goal_pose
            tr_target_pose.x += SET_POSE_ADD_X
            new_position = trans.inverted_transform(tr_target_pose)
            new_goal_pose = new_position
            new_goal_pose.theta = angle_ball_to_target + math.pi
            # ドリブルとキックをオフ
            control_target.kick_power = 0.0
            control_target.dribble_power = 0.0
    else:
        avoid_ball = False
        # tr_target_pose = tr_ball_pose
        # tr_target_pose.x -= -0.2 
        # new_goal_pose = trans.inverted_transform(tr_target_pose)
        new_goal_pose = my_pose
        control_target.kick_power = 0.0
        control_target.dribble_power = 0.0
            
    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target, avoid_ball

# 配置を担当しないロボットは避ける
def avoid_ball_place_line(my_pose, ball_info, goal_pose, control_target):

    angle_ball2goal = tool.get_angle(ball_info.pose, goal_pose)

    trans = tool.Trans(ball_info.pose, angle_ball2goal)
    tr_my_pose = trans.transform(my_pose)
    tr_goal_pose = trans.transform(goal_pose)

    # ライン上にいるやつは避ける
    if -BALL_MARGIN_DIST < tr_my_pose.x < tr_goal_pose.x + BALL_MARGIN_DIST and \
        -BALL_MARGIN_DIST < tr_my_pose.y < BALL_MARGIN_DIST:
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
