# coding: UTF-8


import math
import cmath
import numpy
import sys,os

from geometry_msgs.msg import Pose2D

sys.path.append(os.pardir)
from field import Field

def distance_2_poses(pose1, pose2):
    # 2点間の距離を取る
    # pose.theta は使用しない

    diff_pose = Pose2D()

    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return math.hypot(diff_pose.x, diff_pose.y)


def is_close(pose1, pose2, threshold):
    # 2つの姿勢が近いかどうかを判定する
    result = False

    if math.fabs(pose1.x - pose2.x) < threshold.x:
        if math.fabs(pose1.y - pose2.y) < threshold.y:
            if math.fabs(angle_normalize(pose1.theta - pose2.theta)) < threshold.theta:
                    result = True

    return result


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

def get_intersection(pose1, pose2, pose3, pose4):
    # get intersection of line1(pose1, pose2) and line2(pose3, pose4)
    # reference:http://imagingsolution.blog107.fc2.com/blog-entry-137.html

    s1 = ((pose4.x - pose3.x) * (pose1.y - pose3.y) \
            - (pose4.y - pose3.y) * (pose1.x - pose3.x)) / 2.0

    s2 = ((pose4.x - pose3.x) * (pose3.y - pose2.y) \
            - (pose4.y - pose3.y) * (pose3.x - pose2.x)) / 2.0

    try:
        coefficient = s1 / (s1 + s2)
    except ZeroDivisionError:
        return None

    output = Pose2D(0,0,0)
    output.x = pose1.x + (pose2.x - pose1.x) * coefficient
    output.y = pose1.y + (pose2.y - pose1.y) * coefficient

    return output

def is_in_defence_area(pose, team='our'):
    PENALTY_UPPER_FRONT = Field.penalty_pose(team, 'upper_front')
    PENALTY_LOWER_FRONT = Field.penalty_pose(team, 'lower_front')

    pose_is_in_area = False

    if team == 'our':
        # 自チームディフェンスエリアに入っているか
        if pose.x < PENALTY_UPPER_FRONT.x \
                and pose.y < PENALTY_UPPER_FRONT.y \
                and pose.y > PENALTY_LOWER_FRONT.y:
            pose_is_in_area = True
    else:
        # 相手チームディフェンスエリアに入っているか
        if pose.x > PENALTY_UPPER_FRONT.x \
                and pose.y < PENALTY_UPPER_FRONT.y \
                and pose.y > PENALTY_LOWER_FRONT.y:
            pose_is_in_area = True

    return pose_is_in_area


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


