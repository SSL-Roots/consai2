#!/usr/bin/env python2
# coding: UTF-8


import rospy
import math
from consai2_msgs.msg import VisionGeometry
from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import DecodedReferee
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import referee_wrapper as ref
import avoidance
from actions import tool, defense, offense, goalie, normal, ball_placement
from field import Field
from observer import Observer
import role

class RobotNode(object):
    def __init__(self, robot_id):

        self._MY_ID = robot_id

        self._control_target = ControlTarget()
        self._control_target.robot_id = robot_id
        self._control_target.control_enable = False

        self._my_pose = Pose2D()
        self._my_velocity = Pose2D()

        self._is_attacker = False
        self._is_goalie = False

        # 0 is goalie, 1 is attacker, 2~7 is defense
        self._my_role = 1


    def set_state(self, pose, velocity):
        self._my_pose = pose
        self._my_velocity = velocity

    def set_goalie(self):
        self._is_goalie = True

    def get_sleep(self):
        # 制御を停止する
        self._control_target.control_enable = False
        return self._control_target

    def get_action(self, referee, obstacle_avoidance, ball_info, robot_info=None, defense_num=0):
        self._control_target.control_enable = True
        remake_path = False # 経路再生成のフラグ TODO:remake_pathを活用する
        avoid_obstacle = True # 障害物回避の経路追加フラグ
        avoid_ball = False # ボール回避の経路追加フラグ
        zone_enable = False

        # パラメータ初期化
        self._control_target.dribble_power = 0.0
        self._control_target.kick_power = 0.0

        if referee.can_move_robot is False or ball_info.disappeared:
            # 移動禁止 or ボールの消失で制御を停止する
            rospy.logdebug("HALT")
            self._control_target, remake_path= normal.stop(self._control_target)
            avoid_obstacle = False # 障害物回避しない

        elif referee.is_inplay:
            rospy.logdebug("IN-PLAY")
            zone_enable = True

            if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                if tool.is_in_defense_area(ball_info.pose, 'our'):
                    self._control_target = offense.outside_shoot(
                            self._my_pose, ball_info, self._control_target)
                else:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                avoid_obstacle = False # 障害物回避しない
            elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                if tool.is_in_defense_area(ball_info.pose, 'our'):
                    # ボールが自チームのディフェンスエリアにある場合は行動を変える
                    self._control_target = normal.move_to(
                            self._control_target, Pose2D(0,0,0), ball_info, look_ball=True)
                elif tool.is_in_defense_area(ball_info.pose, 'their'):
                    # ボールが相手チームのディフェンスエリアにある場合は行動を変える
                    self._control_target = normal.keep_x(
                            self._control_target, 
                            Field.penalty_pose('their', 'upper_front').x - 1.0,
                            ball_info)
                else:
                    self._control_target = offense.inplay_shoot(
                            self._my_pose, ball_info, self._control_target)
            else:
                self._control_target = defense.defense_decision(
                        self._my_role, ball_info, self._control_target, 
                        self._my_pose, defense_num, robot_info, zone_enable)

        else:
            if referee.referee_id == ref.REFEREE_ID["STOP"]:
                rospy.logdebug("STOP")
                
                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target = offense.interpose(ball_info,
                            self._control_target, dist_from_target = 0.7)
                    avoid_ball = True
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_KICKOFF_PREPARATION"]:
                rospy.logdebug("OUR_KICKOFF_PREPARATION")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = offense.setplay_shoot(
                            self._my_pose, ball_info, self._control_target,
                            kick_enable = False)
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_KICKOFF_START"]:
                rospy.logdebug("OUR_KICKOFF_START")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = offense.setplay_shoot(
                            self._my_pose, ball_info, self._control_target,
                            kick_enable = True)
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_PENALTY_PREPARATION"]:
                rospy.logdebug("OUR_PENALTY_PREPARATION")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = offense.setplay_shoot(
                            self._my_pose, ball_info, self._control_target,
                            kick_enable = False)
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_PENALTY_START"]:
                rospy.logdebug("OUR_PENALTY_START")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = offense.setplay_shoot(
                            self._my_pose, ball_info, self._control_target,
                            kick_enable = True, penalty=True)
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_DIRECT_FREE"]:
                rospy.logdebug("OUR_DIRECT_FREE")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = offense.setplay_pass(
                            self._my_pose, ball_info, self._control_target,
                            Pose2D(3, 0, 0),
                            receive_enable=True, receiver_role_exist=Observer.role_is_exist(role.ROLE_ID["ROLE_DEFENSE_ZONE_1"]),
                            robot_info=robot_info, direct=True)
                elif self._my_role == role.ROLE_ID["ROLE_DEFENSE_ZONE_1"]:
                    self._control_target = normal.move_to(
                            self._control_target, Pose2D(3,0,0), ball_info, look_ball=True)
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_INDIRECT_FREE"]:
                rospy.logdebug("OUR_INDIRECT_FREE")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = offense.setplay_pass(
                            self._my_pose, ball_info, self._control_target,
                            Pose2D(3, 0, 0),
                            receive_enable=True, receiver_role_exist=Observer.role_is_exist(role.ROLE_ID["ROLE_DEFENSE_ZONE_1"]),
                            robot_info=robot_info)
                elif self._my_role == role.ROLE_ID["ROLE_DEFENSE_ZONE_1"]:
                    self._control_target = normal.move_to(
                            self._control_target, Pose2D(3,0,0), ball_info, look_ball=True)
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["OUR_TIMEOUT"]:
                rospy.logdebug("OUR_TIMEOUT")
                # 自チームのタイムアウトではロボットを停止させる

                self._control_target, remake_path= normal.stop(self._control_target)
                avoid_obstacle = False # 障害物回避しない
            elif referee.referee_id == ref.REFEREE_ID["OUR_BALL_PLACEMENT"]:
                rospy.logdebug("OUR_BALL_PLACEMENT")
                replace_pose = referee.placement_position
                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target, avoid_ball = ball_placement.atk(
                            self._my_pose, ball_info, self._control_target, replace_pose, \
                            robot_info, self._MY_ID)
                elif self._my_role == role.ROLE_ID["ROLE_DEFENSE_GOAL_1"]:
                    self._control_target, avoid_ball = ball_placement.recv(
                            self._my_pose, ball_info, self._control_target, replace_pose, \
                            role.ROLE_ID["ROLE_ATTACKER"], robot_info)
                else:
                    self._control_target, avoid_ball = ball_placement.avoid_ball_place_line(
                            self._my_pose, ball_info, replace_pose, self._control_target)
            elif referee.referee_id == ref.REFEREE_ID["THEIR_KICKOFF_PREPARATION"] \
                    or referee.referee_id == ref.REFEREE_ID["THEIR_KICKOFF_START"]:
                rospy.logdebug("THEIR_KICKOFF")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target = offense.interpose(ball_info,
                            self._control_target, dist_from_target = 0.6)
                    avoid_ball = True
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["THEIR_PENALTY_PREPARATION"] \
                    or referee.referee_id == ref.REFEREE_ID["THEIR_PENALTY_START"]:
                rospy.logdebug("THEIR_PENALTY")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                else:
                    self._control_target, remake_path = normal.make_line(
                            self._my_role, ball_info, self._control_target,
                            start_x=-5, start_y=-3, add_x=0.4, add_y=0)
            elif referee.referee_id == ref.REFEREE_ID["THEIR_DIRECT_FREE"]:
                rospy.logdebug("THEIR_DIRECT")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target = offense.interpose(ball_info,
                            self._control_target, dist_from_target = 0.6)
                    avoid_ball = True
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info,zone_enable=True)
            elif referee.referee_id == ref.REFEREE_ID["THEIR_INDIRECT_FREE"]:
                rospy.logdebug("THEIR_INDIRECT")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target = offense.interpose(ball_info,
                            self._control_target, dist_from_target = 0.6)
                    avoid_ball = True
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info, zone_enable=True)
            elif referee.referee_id == ref.REFEREE_ID["THEIR_TIMEOUT"]:
                rospy.logdebug("THEIR_TIMEOUT")

                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    self._control_target = offense.interpose(ball_info,
                            self._control_target, dist_from_target = 0.6)
                    avoid_ball = True
                else:
                    self._control_target = defense.defense_decision(
                            self._my_role, ball_info, self._control_target, 
                            self._my_pose, defense_num, robot_info)
            elif referee.referee_id == ref.REFEREE_ID["THEIR_BALL_PLACEMENT"]:
                rospy.logdebug("THEIR_BALL_PLACEMENT")
                replace_pose = referee.placement_position
                if self._my_role == role.ROLE_ID["ROLE_GOALIE"]:
                    self._control_target = goalie.interpose(
                            ball_info, robot_info, self._control_target)
                    avoid_obstacle = False # 障害物回避しない
                # elif self._my_role == role.ROLE_ID["ROLE_ATTACKER"]:
                    # self._control_target = offense.interpose(ball_info,
                            # self._control_target, dist_from_target = 0.6)
                    # avoid_ball = True
                else:
                    self._control_target, avoid_ball = ball_placement.avoid_ball_place_line(
                            self._my_pose, ball_info, replace_pose, self._control_target,
                            force_avoid=True)
                    # self._control_target = defense.defense_decision(
                            # self._my_role, ball_info, self._control_target, 
                            # self._my_pose, defense_num, robot_info)

        # 障害物回避の経路作成
        if avoid_obstacle:
            self._control_target.path = obstacle_avoidance.add_path(
                    self._control_target.path, self._my_pose, avoid_ball)

        return self._control_target

class Game(object):
    def __init__(self):
        QUEUE_SIZE = 1

        self._FAR_DISTANCE = 1e+10
        self._OUR_COLOR = rospy.get_param('consai2_description/our_color', 'blue')
        self._MAX_ID = rospy.get_param('consai2_description/max_id', 15)
        self._GOALIE_ID = rospy.get_param('consai2_description/goalie_id', 0)
        self._THEIR_COLOR = 'yellow'
        if self._OUR_COLOR == 'yellow':
            self._THEIR_COLOR = 'blue'

        self._robot_node = []
        for robot_id in range(self._MAX_ID + 1):
            self._robot_node.append(RobotNode(robot_id))
            # ゴーリーを割り当てる
            # ゴーリーはconsai2起動時のみにしか変更しない
            if robot_id == self._GOALIE_ID:
                self._robot_node[robot_id].set_goalie()

        self._roledecision = role.RoleDecision(self._MAX_ID, self._GOALIE_ID)

        self._sub_geometry = rospy.Subscriber(
                'vision_receiver/raw_vision_geometry', VisionGeometry,
                self._callback_geometry, queue_size=1)

        self._decoded_referee = DecodedReferee()
        self._sub_decoded_referee = rospy.Subscriber(
                'referee_wrapper/decoded_referee', DecodedReferee, 
                self._callback_referee, queue_size=1)

        self._ball_info = BallInfo()
        self._sub_ball_info = rospy.Subscriber(
                'vision_wrapper/ball_info', BallInfo,
                self._callback_ball_info, queue_size=1)

        self._robot_info = {'our':[],'their':[]}
        self._subs_robot_info = {'our':[],'their':[]}

        self._pubs_control_target = []

        for robot_id in range(self._MAX_ID + 1):
            # 末尾に16進数の文字列をつける
            topic_id = hex(robot_id)[2:]

            self._robot_info['our'].append(RobotInfo())
            self._robot_info['their'].append(RobotInfo())

            topic_name = 'vision_wrapper/robot_info_' + self._OUR_COLOR + '_' + topic_id
            sub_robot_info = rospy.Subscriber(topic_name, RobotInfo, 
                    self._callback_our_info, queue_size=QUEUE_SIZE, 
                    callback_args=robot_id)
            self._subs_robot_info['our'].append(sub_robot_info)

            topic_name = 'vision_wrapper/robot_info_' + self._THEIR_COLOR + '_' + topic_id
            sub_robot_info = rospy.Subscriber(topic_name, RobotInfo, 
                    self._callback_their_info, queue_size=QUEUE_SIZE, 
                    callback_args=robot_id)
            self._subs_robot_info['their'].append(sub_robot_info)

            topic_name = 'consai2_game/control_target_' + self._OUR_COLOR+'_' + topic_id
            pub_control_target = rospy.Publisher(topic_name, ControlTarget,
                    queue_size=1)
            self._pubs_control_target.append(pub_control_target)

        # 障害物回避のためのクラス
        self._obstacle_avoidance = avoidance.ObstacleAvoidance()

    def _callback_geometry(self, msg):
        Field.update(msg)

    def _callback_referee(self, msg):
        self._decoded_referee = msg

    def _callback_ball_info(self, msg):
        self._ball_info = msg

    def _callback_our_info(self, msg, robot_id):
        self._robot_info['our'][robot_id] = msg

    def _callback_their_info(self, msg, robot_id):
        self._robot_info['their'][robot_id] = msg

    def update(self):
        Observer.update_ball_is_moving(self._ball_info)
        Observer.update_role_is_exist(self._roledecision._rolestocker._role_is_exist)

        self._roledecision.set_disappeared([i.disappeared for i in self._robot_info['our']])
        if tool.is_in_defense_area(self._ball_info.pose, 'our') is False \
               and Observer.ball_is_moving() is False:
            # ボールが自チームディフェンスエリア外にあり
            # ボールが動いていないとき、アタッカーの交代を考える
            self._roledecision.check_ball_dist([i.pose for i in self._robot_info['our']], self._ball_info)
        self._roledecision.event_observer()
        defense_num = self._roledecision._rolestocker._defense_num


        self._obstacle_avoidance.update_obstacles(self._ball_info, self._robot_info)
        for our_info in self._robot_info['our']:
            robot_id = our_info.robot_id
            target = ControlTarget()
            # ロールの更新
            self._robot_node[robot_id]._my_role = self._roledecision._rolestocker._my_role[robot_id]
            if our_info.disappeared:
                # ロボットが消えていたら停止
                target = self._robot_node[robot_id].get_sleep()
            else:
                # ロボットの状態を更新
                self._robot_node[robot_id].set_state(
                        our_info.pose, our_info.velocity)
                # 目標位置を生成
                target = self._robot_node[robot_id].get_action(
                        self._decoded_referee,
                        self._obstacle_avoidance,
                        self._ball_info,
                        self._robot_info,
                        defense_num)

            self._pubs_control_target[robot_id].publish(target)

def main():
    rospy.init_node('game')

    game = Game()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        game.update()

        r.sleep()


if __name__ == '__main__':
    main()


