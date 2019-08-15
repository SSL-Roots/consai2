#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import math

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

from actions import tool


class Observer(object):
    _moving_speed_threshold = 1.0
    _moving_speed_hysteresis = 0.3
    _ball_is_moving = False

    @classmethod
    def update_ball_is_moving(cls, ball_info):
        velocity = ball_info.velocity
        ball_speed = tool.getSizeFromCenter(velocity)

        if Observer._ball_is_moving == False and \
                ball_speed > Observer._moving_speed_threshold + Observer._moving_speed_hysteresis:
            Observer._ball_is_moving = True
        elif Observer._ball_is_moving == True and \
                ball_speed < Observer._moving_speed_threshold - Observer._moving_speed_hysteresis:
            Observer._ball_is_moving = False

    @classmethod
    def ball_is_moving(cls):
        return Observer._ball_is_moving

