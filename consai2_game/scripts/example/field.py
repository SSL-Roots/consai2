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
    _penalty_dict = {
            'upper_front' : Pose2D(), 'upper_back' : Pose2D(),
            'lower_front' : Pose2D(), 'lower_back' : Pose2D(),}
    _penalty_pose = {
            'our': copy.deepcopy(_penalty_dict), 
            'their': copy.deepcopy(_penalty_dict)}
    _field = {
            'length' : 0,
            'width' : 0}

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

        # ペナルティエリア座標の更新
        Field._penalty_pose['our']['upper_front'] = [Pose2D(i.p2_x, i.p2_y,0) for i in geometry.field_lines if i.name == "LeftPenaltyStretch"][0]
        Field._penalty_pose['our']['upper_back']  = [Pose2D(i.p1_x, i.p1_y,0) for i in geometry.field_lines if i.name == "LeftFieldLeftPenaltyStretch"][0]
        Field._penalty_pose['our']['lower_front'] = [Pose2D(i.p1_x, i.p1_y,0) for i in geometry.field_lines if i.name == "LeftPenaltyStretch"][0]
        Field._penalty_pose['our']['lower_back']  = [Pose2D(i.p1_x, i.p1_y,0) for i in geometry.field_lines if i.name == "LeftFieldRightPenaltyStretch"][0]
        Field._penalty_pose['their']['upper_front'] = [Pose2D(i.p2_x, i.p2_y,0) for i in geometry.field_lines if i.name == "RightPenaltyStretch"][0]
        Field._penalty_pose['their']['upper_back'] = [Pose2D(i.p1_x, i.p1_y,0) for i in geometry.field_lines if i.name == "RightFieldRightPenaltyStretch"][0]
        Field._penalty_pose['their']['lower_front'] = [Pose2D(i.p1_x, i.p1_y,0) for i in geometry.field_lines if i.name == "RightPenaltyStretch"][0]
        Field._penalty_pose['their']['lower_back'] = [Pose2D(i.p1_x, i.p1_y,0) for i in geometry.field_lines if i.name == "RightFieldLeftPenaltyStretch"][0]

        # フィールドサイズ取得
        Field._field['length'] = geometry.field_length
        Field._field['width'] = geometry.field_width

    @classmethod
    def goal_pose(cls, team='our', position='center'):
        return Field._goal_pose[team][position]

    @classmethod
    def penalty_pose(cls, team='our', position='upper_front'):
        return Field._penalty_pose[team][position]

    @classmethod
    def field(cls, param='length'):
        return Field._field[param]

