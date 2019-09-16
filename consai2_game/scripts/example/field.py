# coding: UTF-8

import rospy
import copy
import sys
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionGeometry

class Field(object):
    # Division A仕様をデフォルトとして設定
    _our_goal_dict = {'upper' : Pose2D(-6.0, 0.6, 0), 'center' : Pose2D(-6.0, 0.0, 0), 'lower' : Pose2D(-6.0, -0.6, 0)}
    _their_goal_dict = {'upper' : Pose2D(6.0, 0.6, 0), 'center' : Pose2D(6.0, 0.0, 0), 'lower' : Pose2D(6.0, -0.6, 0)}
    _goal_pose = {
            'our': copy.deepcopy(_our_goal_dict), 
            'their': copy.deepcopy(_their_goal_dict)}
    _our_penalty_dict = {
            'upper_front' : Pose2D(-4.795, 1.2, 0), 'upper_back' : Pose2D(-5.99, 1.2, 0),
            'lower_front' : Pose2D(-4.795, -1.2, 0), 'lower_back' : Pose2D(-5.99, -1.2, 0),}
    _their_penalty_dict = {
            'upper_front' : Pose2D(4.795, 1.2, 0), 'upper_back' : Pose2D(5.99, 1.2, 0),
            'lower_front' : Pose2D(4.795, -1.2, 0), 'lower_back' : Pose2D(5.99, -1.2, 0),}
    _penalty_pose = {
            'our': copy.deepcopy(_our_penalty_dict), 
            'their': copy.deepcopy(_their_penalty_dict)}
    _field = {
            'length' : 12.0,
            'width' : 9.0}
    _geometry_field_lines = {}

    # _penalty_poseとfield_lines紐付ける辞書
    _field_lines_to_penalty_pose = {
        'our_upper_front'  :["LeftPenaltyStretch",1],
        'our_upper_back'   :["LeftFieldLeftPenaltyStretch",0],
        'our_lower_front'  :["LeftPenaltyStretch",0],
        'our_lower_back'   :["LeftFieldRightPenaltyStretch",0],
        'their_upper_front':["RightPenaltyStretch",1],
        'their_upper_back' :["RightFieldRightPenaltyStretch",0],
        'their_lower_front':["RightPenaltyStretch",0],
        'their_lower_back' :["RightFieldLeftPenaltyStretch",0],
    }


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

        # geometry.field_linesに入っている座標データを更新
        for field_lines_info in geometry.field_lines:
            Field._geometry_field_lines[field_lines_info.name] = [Pose2D(field_lines_info.p1_x, field_lines_info.p1_y, 0), Pose2D(field_lines_info.p2_x, field_lines_info.p2_y, 0)] 

        # geometry.field_linesにペナルティエリアに関する情報が入っていれば、更新する
        for team_position in Field._field_lines_to_penalty_pose:
            if Field._field_lines_to_penalty_pose[team_position][0] in Field._geometry_field_lines:
                geometry_field_lines_name = Field._field_lines_to_penalty_pose[team_position][0]
                geometry_field_lines_p_num = Field._field_lines_to_penalty_pose[team_position][1]
                # "team_position"をteamとpositionに分割
                team = team_position.split('_',1)[0]
                position = team_position.split('_',1)[1]
                Field._penalty_pose[team][position] = Field._geometry_field_lines[geometry_field_lines_name][geometry_field_lines_p_num]

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
