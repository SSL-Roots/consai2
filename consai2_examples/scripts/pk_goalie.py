#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import BallInfo
from consai2_msgs.msg import VisionGeometry
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D

class PkGoalie(object):
    def __init__(self):

        self.ball_info = None
        self.vision_geometry = None

        # ボール情報をもらう
        sub_ball_info = rospy.Subscriber('vision_wrapper/ball_info',
            BallInfo, callback_ball_info, queue_size=1)

        # フィールド情報をもらう
        sub_field_info = rospy.Subscriber('vision_receiver/raw_vision_geometry',
            VisionGeometry, _callback_vision_geometry)

        # フィールドサイズを取得
        self.field_length, self.field_width, self.goal_width = get_field_size()

    def _callback_ball_info(self, msg):
        self.ball_info = msg

    def _callback_vision_geometry(self, msg):
        self.vision_geometry = msg

    def get_field_size(self):
        self.field_length = vision_geometry.field_length
        self.field_width = vision_geometry.field_width
        self.goal_width = vision_geometry.goal_width

        return field_length, field_width, goal_width

    def get_control_target(self, my_robot_info, ball_info,
        field_length, field_width, goal_width,
        kazasu_left, kazasu_right):
        control_target = ControlTarget()
        # control_target.goal_velocity = Pose2D(0, 0, 3.14)
        control_target.path.append(Pose2D(-1, 1, 0))

        return control_target

