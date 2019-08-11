# coding: UTF-8

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool

sys.path.append(os.pardir)
from field import Field


# 回避用の関数
def basic_avoid(my_pose, ball_pose, goal_pose, robot_info, angle_to_goal):

    # ロボットからボールの角度
    angle_to_goal = tool.get_angle(my_pose, ball_pose)
    # ロボットからみた座標系に変換する
    trans = tool.Trans(my_pose, angle_to_goal)
    tr_my_pose = trans.transform(my_pose)
    tr_goal_pose = trans.transform(goal_pose)

    # 自分と敵の距離を算出
    dist = []
    tr_their_pose = []
    obst_id = []
    obst_dist = []
    robot_info_their = robot_info['their']
    for i, their_info in enumerate(robot_info_their):
        # ロボットのID
        tr_robot_pose = trans.transform(their_info.pose)
        # ロボットの位置を変換
        tr_their_pose.append(tr_robot_pose) 

        # ロボットが検出できている場合
        if their_info.detected:
            # ロボット間の距離を算出
            dist.append(tool.distance_2_poses(their_info.pose, my_pose))

            # 進路上にロボットが居るか
            threshold_tr_y = 0.3
            margin_tr_x = 0.1
            if tr_my_pose.x + margin_tr_x < tr_robot_pose.x and tr_robot_pose.x < tr_goal_pose.x and \
                    tr_my_pose.y - threshold_tr_y < tr_robot_pose.y and tr_robot_pose.y < tr_my_pose.y + threshold_tr_y:

                # 障害物になるロボットの番号
                obst_id.append(i)
                obst_dist.append(dist[-1])
        else:
            dist.append(100)

    # 進路上に敵が居る
    if 0 < len(obst_id):
        
        min_dist = min(obst_dist)
        min_dist_id = dist.index(min_dist)

        print dist
        print min_dist, obst_id, min_dist_id

        # 一番近い敵ロボットの座標
        robot_pose = robot_info_their[min_dist_id].pose
        tr_robot_pose = trans.transform(robot_pose)
        tr_robot_pose.y -= 0.4
        avoid_pose = trans.inverted_transform(tr_robot_pose)
    else:
        avoid_pose = None

    return avoid_pose

