# coding: UTF-8


import math
from geometry_msgs.msg import Pose2D


def distance_2_poses(pose1, pose2):
    # 2点間の距離を取る
    # pose.theta は使用しない

    diff_pose = Pose2D()

    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return math.hypot(diff_pose.x, diff_pose.y)


def angle_normalize(angle):
    # 角度をpi  ~ -piの範囲に変換する
    while angle > math.pi:
        angle -= 2*math.pi

    while angle < -math.pi:
        angle += 2*math.pi

    return angle
