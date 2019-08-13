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


class ObstacleAvoidance(object):
    def __init__(self):

        # ボール情報の初期化
        self._ball_info = BallInfo()
        # ロボット情報の初期化
        self._robot_info = RobotInfo()

        # 検出の幅
        self._range_tr_y = 0.5
        self._range_tr_x = 0.2

        # 経路生成時の相対的に移動する位置(ロボットから見た座標系)
        self._tr_move_x = 0.2
        self._tr_move_y = 0.5

        
    # フィールドのボールとロボットの情報を更新
    def update_obstacles(self, ball_info, robot_info):
        self._ball_info = ball_info
        self._robot_info = robot_info

    # 中間パスを生成してtarget.pathに追加する
    # ball_avoid_flagでボールも障害物にするか決める
    def add_path(self, target_path, my_pose, ball_avoid_flag=False):
        # 中間パスを生成
        avoid_pose = self.basic_avoid(my_pose, target_path, ball_avoid_flag)

        # パスに追加する
        if avoid_pose is not None:
            target_path.insert(0, avoid_pose)

        return target_path

    # 回避用の関数
    def basic_avoid(self, my_pose, target_path, ball_avoid_flag):

        # 使いやすいよう変数に格納しておく
        goal_pose = target_path[-1]
        ball_pose = self._ball_info.pose

        # ロボットから目標地点の角度
        angle_to_goal = tool.get_angle(my_pose, goal_pose)
        # ロボットからみた座標系に変換する
        trans = tool.Trans(my_pose, angle_to_goal)
        tr_my_pose = trans.transform(my_pose)
        tr_goal_pose = trans.transform(goal_pose)

        # 自分と敵の距離を算出
        dist_our, obst_dist_our, obst_id_our = self.detect_dist_and_id(
                                    trans, my_pose, goal_pose, self._robot_info['our'])
        dist_their, obst_dist_their, obst_id_their = self.detect_dist_and_id(
                                    trans, my_pose, goal_pose, self._robot_info['their'])

        # 進路上のロボットの存在をチェックし、経路を生成する
        if len(obst_id_our) == 0 and len(obst_id_their) == 0:
            avoid_pose = None
            dist_robot2path = 0
        else:
            # 進路上の障害物から避ける経路を生成
            avoid_pose_our, min_dist_our = self.gen_avoid_pose(
                    trans, my_pose, self._robot_info['our'], dist_our, obst_dist_our, obst_id_our)
            avoid_pose_their, min_dist_their = self.gen_avoid_pose(
                    trans, my_pose, self._robot_info['their'], dist_their, obst_dist_their, obst_id_their)

           # 敵と味方どちらが近い障害物か比較して近い方を優先する
            if min_dist_our < min_dist_their:
                avoid_pose = avoid_pose_our
                dist_robot2path = min_dist_our
            else:
                avoid_pose = avoid_pose_their
                dist_robot2path = min_dist_their

        # ボールを避けるフラグがTureのときにavoid_poseと比較する
        if ball_avoid_flag:
            dist_robot2ball = tool.distance_2_poses(ball_pose, my_pose)

            # 障害物の左右に中間パスを生成
            tr_avoid_pose_right = trans.transform(ball_pose)
            tr_avoid_pose_left = trans.transform(ball_pose)
            tr_avoid_pose_right.x += self._tr_move_x
            tr_avoid_pose_right.y -= self._tr_move_y
            tr_avoid_pose_left.x += self._tr_move_x
            tr_avoid_pose_left.y += self._tr_move_y
            # 生成したパスとの距離を算出
            dist_right = tool.distance_2_poses(tr_my_pose, tr_avoid_pose_right)
            dist_left = tool.distance_2_poses(tr_my_pose, tr_avoid_pose_left)
    
            # 一番近い敵ロボットの座標の横にパスを生成
            if dist_left < dist_right:
                tr_avoid_pose = tr_avoid_pose_left
            else:
                tr_avoid_pose = tr_avoid_pose_right
           
            ball_avoid_pose = trans.inverted_transform(tr_avoid_pose)

            ball_detect_flag = self.detect_object_on_trajectory(trans, my_pose, goal_pose, ball_pose)

            if ball_detect_flag == True:
                if avoid_pose is None:
                    avoid_pose = ball_avoid_pose
                elif dist_robot2ball < dist_robot2path:
                    avoid_pose = ball_avoid_pose

        return avoid_pose

    # 進路上に居る距離とIDを検索
    def detect_dist_and_id(self, trans, my_pose, goal_pose, robot_info_team):

        # ロボット座標の変換
        tr_my_pose = trans.transform(my_pose)
        tr_goal_pose = trans.transform(goal_pose)

        dist = []
        obst_id = []
        obst_dist = []
        for i, info in enumerate(robot_info_team):
            # ロボットの座標を変換
            tr_robot_pose = trans.transform(info.pose)

            # ロボットが検出できている場合
            if info.disappeared is False:
                # ロボット間の距離を算出
                dist.append(tool.distance_2_poses(info.pose, my_pose))


                # 進路上にロボットが居るか
                if tr_my_pose.x + self._range_tr_x < tr_robot_pose.x and tr_robot_pose.x < tr_goal_pose.x and \
                        tr_my_pose.y - self._range_tr_y < tr_robot_pose.y and tr_robot_pose.y < tr_my_pose.y + self._range_tr_y:

                    # 障害物になるロボットの番号
                    obst_id.append(i)
                    obst_dist.append(dist[-1])
            else:
                dist.append(100)

        return dist, obst_dist, obst_id

    def detect_object_on_trajectory(self, trans, my_pose, goal_pose, object_pose):
        
        tr_my_pose = trans.transform(my_pose)
        tr_object_pose = trans.transform(object_pose)
        tr_goal_pose = trans.transform(goal_pose)

        if tr_my_pose.x + self._range_tr_x < tr_object_pose.x and tr_object_pose.x < tr_goal_pose.x and \
                tr_my_pose.y - self._range_tr_y < tr_object_pose.y and tr_object_pose.y < tr_my_pose.y + self._range_tr_y:
            flag = True
        else:
            flag = False
        return flag

    # 避ける位置を生成
    def gen_avoid_pose(self, trans, my_pose, robot_info_team, dist, obst_dist, obst_id):
    
        if 0 < len(obst_id):
            min_dist = min(obst_dist)
            min_dist_id = dist.index(min_dist)
    
            tr_my_pose = trans.transform(my_pose)
    
            # 障害物の左右に中間パスを生成
            robot_pose = robot_info_team[min_dist_id].pose
            tr_robot_pose = trans.transform(robot_pose)
            tr_avoid_pose_right = trans.transform(robot_pose)
            tr_avoid_pose_left = trans.transform(robot_pose)
            tr_avoid_pose_right.x += self._tr_move_x
            tr_avoid_pose_right.y -= self._tr_move_y
            tr_avoid_pose_left.x += self._tr_move_x
            tr_avoid_pose_left.y += self._tr_move_y
            # 生成したパスとの距離を算出
            dist_right = tool.distance_2_poses(tr_my_pose, tr_avoid_pose_right)
            dist_left = tool.distance_2_poses(tr_my_pose, tr_avoid_pose_left)
    
            # 一番近い敵ロボットの座標の横にパスを生成
            if dist_left < dist_right:
                tr_avoid_pose = tr_avoid_pose_left
            else:
                tr_avoid_pose = tr_avoid_pose_right
           
            avoid_pose = trans.inverted_transform(tr_avoid_pose)
        else:
            avoid_pose = Pose2D(100, 100, 0)
            min_dist = (tool.distance_2_poses(avoid_pose, my_pose))
    
        return avoid_pose, min_dist
    
    
