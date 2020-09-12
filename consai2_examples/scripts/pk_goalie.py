#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import BallInfo
from consai2_msgs.msg import VisionGeometry
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

import math

class PkGoalie(object):
    def __init__(self):
        # ゴールライン上から一定距離[m]前を守るための変数
        self.MARGIN_DIST_X = 0.2
        # ボールが動いていると判定する速度[m/s]
        self.MOVE_BALL_VELOCITY_THRESHOLD =  0.5

    # ゴーリーの目標位置生成
    def get_control_target(self, my_robot_info, ball_info,
        field_length, field_width, goal_width,
        kazasu_left, kazasu_right,
        level=1):

        control_target = ControlTarget()

        # 新しい自身の位置を生成
        new_my_pose = Pose2D(0, 0, 0)
        new_my_pose.x = -field_length / 2 + self.MARGIN_DIST_X

        # ゴール位置を生成
        goal_pose = Pose2D(-field_length / 2, 0, 0)

        # ボールの速度
        ball_velocity_x = ball_info.velocity.x
        ball_velocity_y = ball_info.velocity.y
        ball_velocity = math.hypot(ball_velocity_x, ball_velocity_y)

        # ボールの進む角度
        angle_ball = math.atan2(ball_velocity_y, ball_velocity_x)

        # ボールの進む変化量を計算（方向を考慮した単位量）
        var_ball_velocity_x = math.cos(angle_ball) 
        var_ball_velocity_y = math.sin(angle_ball) 

        # ボールの位置を抽出
        ball_pose = ball_info.pose

        # ボールの次の予測位置を取得
        ball_pose_next = Pose2D(
            ball_pose.x + var_ball_velocity_x, ball_pose.y + var_ball_velocity_y, 0) 

        # ボールがゴールに入っている場合はその場のy座標を代入
        if ball_pose.x < goal_pose.x: 
            new_my_pose.y = my_robot_info.pose.y
        # ゴールされていなければゴーリの新しいy座標を生成
        else:
            # level1
            if level == 1:
                # ボールのy座標と同じ位置に移動
                new_my_pose.y = ball_pose.y
            # lebel2: ボールが一定速度以上かつ向かってくる場合
            elif level == 2 and self.MOVE_BALL_VELOCITY_THRESHOLD < ball_velocity and ball_velocity_x < 0:
                # ボールの進路に関する直線の傾きと切片を算出
                slope, intercept = self._get_line_parameters(
                    ball_pose, ball_pose_next)
                # y座標を算出
                new_my_pose.y = slope * new_my_pose.x + intercept

            # ボールが止まっている場合など
            else:
                # ボールとゴールを結ぶ直線の傾きと切片を算出
                slope, intercept = self._get_line_parameters(
                    ball_pose, goal_pose)
                # y座標を算出
                new_my_pose.y = slope * new_my_pose.x + intercept

        # ゴール幅からはみ出ないように制限する
        if goal_width / 2 < new_my_pose.y:
            new_my_pose.y = goal_width / 2
        elif new_my_pose.y < -goal_width / 2:
            new_my_pose.y = -goal_width / 2

        # 新しいパスを追加
        control_target.path.append(new_my_pose)

        return control_target

    # 2点の座標から直線の傾きと切片を算出, 
    def _get_line_parameters(self, pose1, pose2):

        x1 = pose1.x
        y1 = pose1.y
        x2 = pose2.x
        y2 = pose2.y

        # 0になるとエラーになるので適当な小数を入れる
        if x1 - x2 == 0:
            x1 += 1e-12
        
        # 傾きの算出
        slope = (y2 - y1) / (x2 - x1)
        # 切片の算出
        intercept = y2 - slope * x2
        
        return slope, intercept

