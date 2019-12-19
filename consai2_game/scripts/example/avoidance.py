# coding: UTF-8

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
from actions import tool

sys.path.append(os.pardir)
from field import Field

# 障害物回避
class ObstacleAvoidance(object):
    def __init__(self):

        # ボール情報の初期化
        self._ball_info = BallInfo()
        # ロボット情報の初期化
        self._robot_info = RobotInfo()

        # 障害物検出の幅[m]
        self._range_tr_y = 0.5
        self._range_tr_x = 0.2

        # 経路生成時の相対的に移動する位置(ロボットから見た座標系)
        self._tr_move_x = 0.2
        self._tr_move_y = 0.5

    # フィールドのボールとロボットの情報を更新
    def update_obstacles(self, ball_info, robot_info):
        self._ball_info = ball_info
        self._robot_info = robot_info

    # 中間パスを生成しパスに追加する
    # ball_avoid_flagでボールも障害物にするか決める
    def add_path(self, target_path, my_pose, ball_avoid_flag=False):

        # 新しいパス生成用の変数
        new_target_path = []
        new_target_path.append(target_path[-1]) 

        # 中間パスを生成
        avoid_pose = self._basic_avoid(my_pose, new_target_path, ball_avoid_flag)

        # IREXの（やべぇ）柱を回避する
        avoid_pose = self._avoid_crazy_pillar(my_pose, new_target_path)

        # 中間パスが生成された場合はパスに追加
        if avoid_pose is not None:
            avoid_pose.theta = target_path[-1].theta
            new_target_path.insert(0, avoid_pose)

        return new_target_path

    # IREX会場の（頭おかしい）柱を回避する
    def _avoid_crazy_pillar(self, my_pose, target_path):
        PILLARS = [Pose2D(-3.3, 1.75, 0), Pose2D(3.3, 1.75, 0)]

        THRESHOLD_DIST = 0.3 # meters 回避判定の距離
        AVOID_DIST = 0.4 # meters 回避位置の距離

        target_pose = target_path[-1]
        
        for pillar in PILLARS:
            if tool.distance_2_poses(pillar, target_pose) < THRESHOLD_DIST:
                # angle_to_target = tool.get_angle(pillar, target_pose)
                # trans = tool.Trans(pillar, angle_to_target)
                angle_to_my_pose = tool.get_angle(pillar, my_pose)
                trans = tool.Trans(pillar, angle_to_my_pose)
                avoid_pose = trans.inverted_transform(Pose2D(AVOID_DIST, 0, 0))

                return avoid_pose


    # 回避用の関数
    def _basic_avoid(self, my_pose, target_path, ball_avoid_flag):

        # 使いやすいよう変数に格納しておく
        target_pose = target_path[-1]
        ball_pose = self._ball_info.pose

        # ロボットから目標地点の角度
        angle_i_to_target = tool.get_angle(my_pose, target_pose)
        # ロボットからみた座標系に変換するクラス
        trans = tool.Trans(my_pose, angle_i_to_target)

        # 近いロボットに対して中間パスを生成
        avoid_pose, dist_i_to_robot = self._generate_avoid_pose_to_near_robot(trans, my_pose, target_pose)

        # ボールを避けるフラグがTureのときにすでに生成しているavoid_poseと比較する
        if ball_avoid_flag:

            # ボールが進路上に存在しているか調べる
            ball_detect_flag = self._detect_object_on_trajectory(trans, target_pose, ball_pose)

            # ボールが進路上に存在している場合は処理を行う
            if ball_detect_flag == True:
                dist_i_to_ball = tool.distance_2_poses(ball_pose, my_pose)
                
                # 中間パスがない or ボールの方が近い場合はボールに対して中間パスを生成
                if avoid_pose is None or dist_i_to_ball < dist_i_to_robot:
                    avoid_pose = self._generate_avoid_pose(trans, my_pose, ball_pose)

        return avoid_pose

    # 近いロボットに対して中間パスを生成
    def _generate_avoid_pose_to_near_robot(self, trans, my_pose, target_pose):

        # ロボット中心に座標変換
        tr_my_pose = trans.transform(my_pose)
        tr_target_pose = trans.transform(target_pose)

        # 障害物が見つからない場合の値
        avoid_pose = None
        obst_pose = None
        obst_dist = 100

        # 各ロボットに対して処理を行う
        for robot_info_team in [self._robot_info['our'], self._robot_info['their']]:
            for i, info in enumerate(robot_info_team):
                # ロボットが検出できている場合
                if info.disappeared is False:
                    # 進路上にロボットが居る場合
                    flag = self._detect_object_on_trajectory(trans, target_pose, info.pose)
                    if flag:
                        # 距離が近い場合更新
                        dist = tool.distance_2_poses(info.pose, my_pose)
                        if dist < obst_dist:
                            # 障害物情報の更新
                            obst_dist = dist
                            obst_pose = info.pose

        # 進路上に障害物が存在していた場合は中間パスを生成
        if obst_pose is not None:
            avoid_pose = self._generate_avoid_pose(trans, my_pose, obst_pose)

        return avoid_pose, obst_dist
    
    # 目標位置との間に障害物があるか判定
    def _detect_object_on_trajectory(self, trans, target_pose, object_pose):
        
        # 自分と障害物と目標地点の座標変換
        tr_object_pose = trans.transform(object_pose)
        tr_target_pose = trans.transform(target_pose)

        # 進路上にいる障害物の存在を調べる(しきい値は__init__()で決定)
        flag = False
        if self._range_tr_x < tr_object_pose.x and tr_object_pose.x < tr_target_pose.x and \
                -self._range_tr_y < tr_object_pose.y and tr_object_pose.y < self._range_tr_y:
            flag = True

        return flag
    
    # 障害物を避ける位置を生成
    def _generate_avoid_pose(self, trans, my_pose, obst_pose):
    
        # ロボット中心の座標系に変換
        tr_my_pose = trans.transform(my_pose)

        # 障害物の左右に中間パスを生成
        tr_avoid_pose_right = trans.transform(obst_pose)
        tr_avoid_pose_left = trans.transform(obst_pose)
        tr_avoid_pose_right.x += self._tr_move_x
        tr_avoid_pose_right.y -= self._tr_move_y
        tr_avoid_pose_left.x += self._tr_move_x
        tr_avoid_pose_left.y += self._tr_move_y

        # 生成したパスとの距離を算出
        dist_right = tool.distance_2_poses(tr_my_pose, tr_avoid_pose_right)
        dist_left = tool.distance_2_poses(tr_my_pose, tr_avoid_pose_left)

        # 一番近い敵ロボットの座標の横に回避パスを生成
        if dist_left < dist_right:
            tr_avoid_pose = tr_avoid_pose_left
        else:
            tr_avoid_pose = tr_avoid_pose_right

        # 中間パスをワールド座標系に変換
        avoid_pose = trans.inverted_transform(tr_avoid_pose)
    
        return avoid_pose

