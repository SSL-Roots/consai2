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
from actions import tool, defense, offense, goalie
import role
from field import Field

ROLE_GOALIE = 0
ROLE_ATTACKER = 1
ROLE_DEFENCE_GOAL_1 = 2
ROLE_DEFENCE_GOAL_2 = 3
ROLE_DEFENCE_ZONE_1 = 4
ROLE_DEFENCE_ZONE_2 = 5
ROLE_DEFENCE_ZONE_3 = 6
ROLE_DEFENCE_ZONE_4 = 7
ROLE_NONE = 99

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


    def get_action(self, referee, obstacle_avoidance, ball_info, robot_info=None):
        self._control_target.control_enable = True

        # reset_flag = True
        if referee.can_move_robot is False or ball_info.disappeared:
            # 移動禁止 or ボールの消失で制御を停止する
            self._control_target.control_enable = False

        elif referee.is_inplay:
            rospy.logdebug("IN-PLAY")
            pose = Pose2D()
            # if self._is_goalie:
            if self._my_role == 0:
                self._control_target = goalie.interpose(
                        ball_info, robot_info, self._control_target)
            elif self._my_role == 1:
                # アタッカーならボールに近づく
                self._control_target = offense.simple_kick(self._my_pose, ball_info, self._control_target, kick_power=0.5)
                # 障害物位置を検出し、中間パスの生成と追加を行う
                self._control_target.path = obstacle_avoidance.add_path(self._control_target.path, self._my_pose)
            else:
                # それ以外ならくるくる回る
                # パスを初期化 (あくまでテスト用、本来はパスは消すべきではない)
                self._control_target.path = []
            
            pass
        else:
            if referee.referee_id == ref.REFEREE_ID["STOP"]:
                rospy.logdebug("STOP")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_KICKOFF_PREPARATION"]:
                rospy.logdebug("OUR_KICKOFF_PREPARATION")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_KICKOFF_START"]:
                rospy.logdebug("OUR_KICKOFF_START")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_PENALTY_PREPARATION"]:
                rospy.logdebug("OUR_PENALTY_PREPARATION")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_PENALTY_START"]:
                rospy.logdebug("OUR_PENALTY_START")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_DIRECT_FREE"]:
                rospy.logdebug("OUR_DIRECT_FREE")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_INDIRECT_FREE"]:
                rospy.logdebug("OUR_INDIRECT_FREE")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_TIMEOUT"]:
                rospy.logdebug("OUR_TIMEOUT")
                pass
            elif referee.referee_id == ref.REFEREE_ID["OUR_BALL_PLACEMENT"]:
                rospy.logdebug("OUR_BALL_PLACEMENT")
                pass
            elif referee.referee_id == ref.REFEREE_ID["THEIR_KICKOFF_PREPARATION"] \
                    or referee.referee_id == ref.REFEREE_ID["THEIR_KICKOFF_START"]:
                rospy.logdebug("THEIR_KICKOFF")
                pass
            elif referee.referee_id == ref.REFEREE_ID["THEIR_PENALTY_PREPARATION"] \
                    or referee.referee_id == ref.REFEREE_ID["THEIR_PENALTY_START"]:
                rospy.logdebug("THEIR_PENALTY")
                pass
            elif referee.referee_id == ref.REFEREE_ID["THEIR_DIRECT_FREE"]:
                rospy.logdebug("THEIR_DIRECT")
                pass
            elif referee.referee_id == ref.REFEREE_ID["THEIR_INDIRECT_FREE"]:
                rospy.logdebug("THEIR_INDIRECT")
                pass
            elif referee.referee_id == ref.REFEREE_ID["THEIR_TIMEOUT"]:
                rospy.logdebug("THEIR_TIMEOUT")
                pass
            elif referee.referee_id == ref.REFEREE_ID["THEIR_BALL_PLACEMENT"]:
                rospy.logdebug("THEIR_BALL_PLACEMENT")
                pass


            self._control_target.path = []
            pose = Pose2D()
            pose.x = self._my_pose.x
            pose.y = self._my_pose.y
            pose.theta = self._my_pose.theta + math.radians(30) # くるくる回る

            if self._my_role == ROLE_GOALIE:
                pose.x = -4
                pose.y = 0
            elif self._my_role == ROLE_ATTACKER:
                pose.x = ball_info.pose.x - 0.5
                pose.y = ball_info.pose.y
            elif self._my_role == ROLE_DEFENCE_GOAL_1:
                pose.x = -3.5
                pose.y = 1
            elif self._my_role == ROLE_DEFENCE_GOAL_2:
                pose.x = -3.5
                pose.y = -1
            elif self._my_role == ROLE_DEFENCE_ZONE_1:
                pose.x = -1
                pose.y = 1
            elif self._my_role == ROLE_DEFENCE_ZONE_2:
                pose.x = -1
                pose.y = -1
            elif self._my_role == ROLE_DEFENCE_ZONE_3:
                pose.x = -1
                pose.y = 2
            elif self._my_role == ROLE_DEFENCE_ZONE_4:
                pose.x = -1
                pose.y = -2
            elif self._my_role == ROLE_NONE:
                pose.theta = self._my_pose.theta # まわらない

            self._control_target.path.append(pose)

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
        self._roledecision.set_disappeared([i.disappeared for i in self._robot_info['our']])
        self._roledecision.check_ball_dist([i.pose for i in self._robot_info['our']], self._ball_info)
        self._roledecision.event_observer()

        self._obstacle_avoidance.update_obstacles(self._ball_info, self._robot_info)

        for our_info in self._robot_info['our']:
            robot_id = our_info.robot_id
            target = ControlTarget()
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
                        self._robot_info)

            self._robot_node[robot_id]._my_role = self._roledecision._rolestocker._my_role[robot_id]

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


