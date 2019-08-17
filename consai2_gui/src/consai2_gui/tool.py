# coding: UTF-8


import math
import cmath
import numpy
import sys,os

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


def get_angle(from_pose, to_pose):
    # ワールド座標系でfrom_poseからto_poseを結ぶ直線の角度を得る
    diff_pose = Pose2D()

    diff_pose.x = to_pose.x - from_pose.x
    diff_pose.y = to_pose.y - from_pose.y

    return math.atan2(diff_pose.y, diff_pose.x)


class Trans():
    # 座標系を移動、回転するクラス
    def __init__(self, center , theta):

        normalized_theta = angle_normalize(theta)
        self._c_center = center.x + center.y * 1.0j
        self._c_rotate = cmath.rect(1.0,normalized_theta) 
        self._c_angle = normalized_theta

    def transform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = (c_point - self._c_center) * numpy.conj(self._c_rotate)

        output = Pose2D()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def inverted_transform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = c_point * self._c_rotate + self._c_center

        output = Pose2D()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def transform_angle(self, angle):
        return angle_normalize(angle - self._c_angle)

    def inverted_transform_angle(self, angle):
        return angle_normalize(angle + self._c_angle)

