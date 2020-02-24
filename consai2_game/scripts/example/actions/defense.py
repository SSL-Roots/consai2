# coding: UTF-8
# defense.pyでは、ボールを蹴らないactionを定義する

import math
import rospy
import sys,os

from geometry_msgs.msg import Pose2D

sys.path.append(os.pardir)
from observer import Observer
import role
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


# ボールレシーブ情報保持用のクラス
class Receiving(object):
    _recenving = [False] * role.ZONE_DEFENSE_NUM

    @classmethod
    def update_receiving(cls, zone_id, param):
        Receiving._recenving[zone_id] = param
    @classmethod
    def receiving(cls, zone_id):
        return Receiving._recenving[zone_id]


def update_receive_ball(ball_info, my_pose, zone_id):
    # ボール位置
    ball_pose = ball_info.pose
    # ボールスピード
    ball_vel = ball_info.velocity

    # 受け取れると判断する距離
    _can_receive_dist = 1.0
    # ヒステリシス
    _can_receive_hysteresis = 0.3

    result = False
    target_pose = Pose2D()

    # ボールが動いている
    if Observer.ball_is_moving():
        # ボール速度ベクトルの角度
        angle_velocity = tool.get_angle_from_center(ball_vel)
        trans = tool.Trans(ball_pose, angle_velocity)

        tr_pose = trans.transform(my_pose)
        
        # ボール速度の線と垂直な距離
        fabs_y = math.fabs(tr_pose.y)

        # 受け取れる判定
        if Receiving.receiving(zone_id) == False and \
                fabs_y < _can_receive_dist - _can_receive_hysteresis:
            Receiving.update_receiving(zone_id, True)
        # 受け取れない判定
        elif Receiving.receiving(zone_id) == True and \
                fabs_y > _can_receive_dist + _can_receive_hysteresis:
            Receiving.update_receiving(zone_id, False)

        # 受け取れるかつボールが向かう方向にいる
        if Receiving.receiving(zone_id) and tr_pose.x > 0.0:
            tr_pose.y = 0.0
            inv_pose = trans.inverted_transform(tr_pose)
            angle_to_ball = tool.get_angle(inv_pose, ball_pose)
            target_pose = Pose2D(inv_pose.x, inv_pose.y, angle_to_ball)
            result = True

    return result, target_pose
