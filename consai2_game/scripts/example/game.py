#!/usr/bin/env python2
# coding: UTF-8


import rospy
import math
from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import DecodedReferee
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import referee_wrapper as ref
from actions import tool, defense


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


    def set_state(self, pose, velocity):
        self._my_pose = pose
        self._my_velocity = velocity

    def set_attacker(self, is_attacker):
        self._is_attacker = is_attacker

    def set_goalie(self):
        self._is_goalie = True


    def get_sleep(self):
        # 制御を停止する
        self._control_target.control_enable = False
        return self._control_target


    def get_action(self, referee, ball_info):
        self._control_target.control_enable = True

        if referee.referee_id == ref.REFEREE_ID["HALT"] or \
                ball_info.disappeared:
            # HALT or ボールの消失で制御を停止する
            self._control_target.control_enable = False

        elif referee.referee_id == ref.REFEREE_ID["STOP"]:

            pose = Pose2D()
            if self._is_goalie:
                # ゴーリーならボールの裏に回る

                # パスを初期化 (あくまでテスト用、本来はパスは消すべきではない)
                self._control_target.path = []
                pose.x = self._my_pose.x
                pose.y = self._my_pose.y
                pose.theta = self._my_pose.theta + math.radians(60) # くるくる回る
                self._control_target.path.append(pose)

            elif self._is_attacker:
                # アタッカーならボールに近づく
                self._control_target = defense.interpose(
                        ball_info, self._control_target, dist_from_target=0.6)
                # rospy.loginfo(self._control_target)
            else:
                # それ以外ならくるくる回る

                # パスを初期化 (あくまでテスト用、本来はパスは消すべきではない)
                self._control_target.path = []
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

        self._FAR_DISTANCE = 1e+10
        self._OUR_COLOR = rospy.get_param('consai2_description/our_color', 'blue')
        self._MAX_ID = rospy.get_param('consai2_description/max_id', 15)
        self._GOALIE_ID = rospy.get_param('consai2_description/goalie_id', 0)
        self._THEIR_COLOR = 'yellow'
        if self._OUR_COLOR == 'yellow':
            self._THEIR_COLOR = 'blue'


        self._robot_node = []
        self._dist_to_ball = {} # ロボットからボールまでの距離
        self._attacker_id = 0 # アタッカーID
        for robot_id in range(self._MAX_ID + 1):
            self._robot_node.append(RobotNode(robot_id))
            # ゴーリーを割り当てる
            # ゴーリーはconsai2起動時のみにしか変更しない
            if robot_id == self._GOALIE_ID:
                self._robot_node[robot_id].set_goalie()

            self._dist_to_ball[robot_id] = self._FAR_DISTANCE


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


    def _callback_referee(self, msg):
        self._decoded_referee = msg

    def _callback_ball_info(self, msg):
        self._ball_info = msg

    def _callback_our_info(self, msg, robot_id):
        self._robot_info['our'][robot_id] = msg

    def _callback_their_info(self, msg, robot_id):
        self._robot_info['their'][robot_id] = msg


    def _i_am_attacker(self, robot_id, new_robot_pose):
        # ロボットとボールの距離を計算して、一番ボールに近ければTrue
        MARGIN = 0.5 # meter

        # ロボットとボールの距離を計算
        # ボールが消える可能性を考慮して、last_detection_poseを使う
        dist = tool.distance_2_poses(new_robot_pose, 
                self._ball_info.last_detection_pose)
        # 距離の更新
        self._dist_to_ball[robot_id] = dist

        # アタッカーIDが自分自身であれば、他と比較せずにTrueを返す
        if self._attacker_id == robot_id:
            return True

        # アタッカーのボールロボット間距離を取得
        # マージンを設けて、アタッカーの切り替わりの発振を防ぐ
        closest_dist = self._dist_to_ball[self._attacker_id] - MARGIN

        if dist < closest_dist:
            self._attacker_id = robot_id
            return True
        else:
            return False


    def update(self):
        for our_info in self._robot_info['our']:
            robot_id = our_info.robot_id
            target = ControlTarget()
            if our_info.disappeared:
                # ロボットが消えていたら停止
                target = self._robot_node[robot_id].get_sleep()

                # ボールとの距離を初期化
                self._dist_to_ball[robot_id] = self._FAR_DISTANCE
        
            else:
                # アタッカー情報をセット
                self._robot_node[robot_id].set_attacker(
                        self._i_am_attacker(robot_id, our_info.pose))

                # ロボットの状態を更新
                self._robot_node[robot_id].set_state(
                        our_info.pose, our_info.velocity)
                # 目標位置を生成
                target = self._robot_node[robot_id].get_action(
                        self._decoded_referee,
                        self._ball_info)

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


