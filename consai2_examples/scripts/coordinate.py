#!/usr/bin/env python2
# coding: UTF-8

import math
import tool

from geometry_msgs.msg import Pose2D

RobotRadius = 0.09
BallRadius = 0.0215

OUR_GOAL_X = -6
OUR_GOAL_Y = 0
FRONT_GOAL_X = 6
FRONT_GOAL_Y = 0

class Coordinate(object):
    # Coordinateクラスは、フィールド状況をもとに移動目標位置、目標角度を生成する
    # Coordinateクラスには、移動目標の生成方法をsetしなければならない
    # Coordinateクラスのposeが生成された移動目標である

    def __init__(self):
        self.pose = Pose2D() # pos_x, pos_y, thta

        self._robot_pose = Pose2D()
        self._ball_pose = Pose2D()
        self.target_pose = Pose2D()

        self._base = None # string data
        self._target = None # string data

        # approach to shoot
        self._pose_max = Pose2D()
        self._role_is_lower_side = False
        self._role_pose_hystersis = 0.1
        self._tuning_param_x = 0.3
        self._tuning_param_y = 0.3
        self._tuning_param_pivot_y = 0.1
        self._tuning_angle = 30.0 * math.pi / 180.0  # 0 ~ 90 degree, do not edit 'math.pi / 180.0'

        self._pose_max.x = BallRadius + self._tuning_param_x
        self._pose_max.y = BallRadius + RobotRadius + self._tuning_param_y

        self.approach_state = 0

    def _update_robot_pose(self, robot_pose):
        self._robot_pose = robot_pose

    def _update_ball_pose(self, ball_pose):
        self._ball_pose = ball_pose

    def get_target_pose(self):
        return self.target_pose

    def _update_approach_to_shoot(self):
        # Reference to this idea
        # http://wiki.robocup.org/images/f/f9/Small_Size_League_-_RoboCup_2014_-_ETDP_RoboDragons.pdf

        _target_pose = Pose2D(FRONT_GOAL_X, FRONT_GOAL_Y,0)
        _role_pose = self._robot_pose

        if _target_pose is None or _role_pose is None:
            return False

        # ボールからターゲットを見た座標系で計算する
        angle_ball_to_target = tool.getAngle(self._ball_pose, _target_pose)
        trans = tool.Trans(self._ball_pose, angle_ball_to_target)
        tr_role_pose = trans.transform(_role_pose)

        # tr_role_poseのloser_side判定にヒステリシスをもたせる
        if self._role_is_lower_side == True and \
                tr_role_pose.y > self._role_pose_hystersis:
            self._role_is_lower_side = False

        elif self._role_is_lower_side == False and \
                tr_role_pose.y < - self._role_pose_hystersis:
            self._role_is_lower_side = True

        if self._role_is_lower_side:
            tr_role_pose.y *= -1.0


        tr_approach_pose = Pose2D()
        if tr_role_pose.x > 0:
            # 1.ボールの斜め後ろへ近づく
            self.approach_state = 1

            # copysign(x,y)でyの符号に合わせたxを取得できる
            tr_approach_pose = Pose2D(
                    -self._pose_max.x,
                    math.copysign(self._pose_max.y, tr_role_pose.y), 
                    0)

        else:
            # ボール裏へ回るためのピボットを生成
            pivot_pose = Pose2D(0, self._tuning_param_pivot_y, 0)
            angle_pivot_to_role = tool.getAngle(pivot_pose,tr_role_pose)

            limit_angle = self._tuning_angle + math.pi * 0.5

            if tr_role_pose.y > self._tuning_param_pivot_y and \
                    angle_pivot_to_role < limit_angle:
                # 2.ボール後ろへ回りこむ
                self.approach_state = 2
            
                diff_angle = tool.normalize(limit_angle - angle_pivot_to_role)
                decrease_coef = diff_angle / self._tuning_angle
            
                tr_approach_pose = Pose2D(
                        -self._pose_max.x,
                        self._pose_max.y * decrease_coef, 
                        0)
            
            else:
                # 3.ボールに向かう
                self.approach_state = 3

                diff_angle = tool.normalize(angle_pivot_to_role - limit_angle)
                approach_coef = diff_angle / (math.pi * 0.5 - self._tuning_angle)
            
                if approach_coef > 1.0:
                    approach_coef = 1.0

                pos_x = approach_coef * (2.0 * BallRadius 
                        - self._tuning_param_x) + self._tuning_param_x
            
                tr_approach_pose = Pose2D(-pos_x, 0, 0)

        # 上下反転していたapproach_poseを元に戻す
        if self._role_is_lower_side:
            tr_approach_pose.y *= -1.0

        self.pose = trans.invertedTransform(tr_approach_pose)
        self.pose.theta = angle_ball_to_target

        self.target_pose = self.pose
        
        return True


