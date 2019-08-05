#!/usr/bin/env python2
# coding: UTF-8


import rospy
import math
from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import DecodedReferee
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import referee_wrapper as ref


class RobotNode(object):
    def __init__(self, robot_id):

        self._MY_ID = robot_id

        self._control_target = ControlTarget()
        self._control_target.robot_id = robot_id
        self._control_target.control_enable = False

        self._my_pose = Pose2D()
        self._my_velocity = Pose2D()


    def set_state(self, pose, velocity):
        self._my_pose = pose
        self._my_velocity = velocity


    def get_sleep(self):
        # 制御を停止する
        self._control_target.control_enable = False
        return self._control_target


    def get_action(self, referee):
        self._control_target.control_enable = True

        if referee.referee_id == ref.REFEREE_ID["HALT"]:
            # 制御を停止する
            self._control_target.control_enable = False

        elif referee.referee_id == ref.REFEREE_ID["STOP"]:
            # パスを初期化 (あくまでテスト用、本来はパスは消すべきではない)
            self._control_target.path = []
            pose = Pose2D()
            pose.x = self._my_pose.x
            pose.y = self._my_pose.y
            pose.theta = self._my_pose.theta + math.radians(30) # くるくる回る
            self._control_target.path.append(pose)

        else:
            # 制御を停止する
            self._control_target.control_enable = False

        return self._control_target


class Game(object):
    def __init__(self):
        QUEUE_SIZE = 1

        self._OUR_COLOR = rospy.get_param('consai2_description/our_color', 'blue')
        self._MAX_ID = rospy.get_param('consai2_description/max_id', 15)
        self._THEIR_COLOR = 'yellow'
        if self._OUR_COLOR == 'yellow':
            self._THEIR_COLOR = 'blue'

        self._robot_node = []

        self._decoded_referee = DecodedReferee()
        self._sub_decoded_referee = rospy.Subscriber(
                'referee_wrapper/decoded_referee', DecodedReferee, 
                self._callback_referee, queue_size=1)

        self._robot_info = {'our':[],'their':[]}
        self._subs_robot_info = {'our':[],'their':[]}

        self._pubs_control_target = []

        for robot_id in range(self._MAX_ID + 1):
            # 末尾に16進数の文字列をつける
            topic_id = hex(robot_id)[2:]

            self._robot_info['our'].append(RobotInfo())
            self._robot_info['their'].append(RobotInfo())
            self._robot_node.append(RobotNode(robot_id))

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


    def _callback_referee(self, msg):
        self._decoded_referee = msg

    def _callback_our_info(self, msg, robot_id):
        self._robot_info['our'][robot_id] = msg

    def _callback_their_info(self, msg, robot_id):
        self._robot_info['their'][robot_id] = msg


    def update(self):
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
                target = self._robot_node[robot_id].get_action(self._decoded_referee)

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


