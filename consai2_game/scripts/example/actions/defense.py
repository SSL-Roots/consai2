# coding: UTF-8
# defense.pyでは、ボールを蹴らないactionを定義する

import math
import rospy
import sys,os

from geometry_msgs.msg import Pose2D

sys.path.append(os.pardir)
import tool


def get_avoid_ball_pose(ball_pose, target_pose):
    # target_poseがボールの半径0.5 m以内にある場合、
    # ボールから離れたtarget_poseを生成する

    THRESHOLD_DIST = 0.5 # meters
    AVOID_DIST = 0.6 # meters

    avoid_target_pose = target_pose

    if tool.distance_2_poses(ball_pose, target_pose) < THRESHOLD_DIST:
        angle_to_target = tool.get_angle(ball_pose, target_pose)
        trans = tool.Trans(ball_pose, angle_to_target)
        avoid_target_pose = trans.inverted_transform(Pose2D(AVOID_DIST, 0, 0))
        # 目標角度を再設定
        avoid_target_pose.theta = target_pose.theta

    return avoid_target_pose

