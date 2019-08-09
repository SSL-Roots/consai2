# coding: UTF-8

import rospy
import copy
import sys
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionGeometry

class Field(object):

    _goal_dict = {'upper' : Pose2D(), 'center' : Pose2D(), 'lower' : Pose2D()}
    _goal_pose = {
            'our': copy.deepcopy(_goal_dict), 
            'their': copy.deepcopy(_goal_dict)}

    @classmethod
    def update(cls, geometry):
        half_length = geometry.field_length * 0.5
        half_width = geometry.field_width * 0.5
        half_goal_width = geometry.goal_width * 0.5

        # ゴール座標の更新
        Field._goal_pose['our']['upper'] = Pose2D(-half_length, half_goal_width, 0)
        Field._goal_pose['our']['center'] = Pose2D(-half_length, 0, 0)
        Field._goal_pose['our']['lower'] = Pose2D(-half_length, -half_goal_width, 0)
        Field._goal_pose['their']['upper'] = Pose2D(half_length, half_goal_width, 0)
        Field._goal_pose['their']['center'] = Pose2D(half_length, 0, 0)
        Field._goal_pose['their']['lower'] = Pose2D(half_length, -half_goal_width, 0)

    @classmethod
    def goal_pose(cls, team='our', position='center'):
        return Field._goal_pose[team][position]


