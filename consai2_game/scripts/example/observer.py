#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import random

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

from actions import tool
import role


class Observer(object):
    _moving_speed_threshold = 1.0
    _moving_speed_hysteresis = 0.3
    _ball_is_moving = False
    _role_is_exist = [False] * (len(role.ROLE_ID) + 1)
    _random_zero_one = random.randint(0,1)

    @classmethod
    def update_ball_is_moving(cls, ball_info):
        velocity = ball_info.velocity
        ball_speed = tool.get_size_from_center(velocity)

        if Observer._ball_is_moving == False and \
                ball_speed > Observer._moving_speed_threshold + Observer._moving_speed_hysteresis:
            Observer._ball_is_moving = True
        elif Observer._ball_is_moving == True and \
                ball_speed < Observer._moving_speed_threshold - Observer._moving_speed_hysteresis:
            Observer._ball_is_moving = False

    @classmethod
    def update_role_is_exist(cls, role_is_exist):
        Observer._role_is_exist = role_is_exist

    @classmethod
    def ball_is_moving(cls):
        return Observer._ball_is_moving

    @classmethod
    def role_is_exist(cls, role_id):
        return Observer._role_is_exist[role_id]

    @classmethod
    def update_random_zero_one(cls):
        Observer._random_zero_one = random.randint(0,1)

    @classmethod
    def random_zero_one(cls):
        return Observer._random_zero_one