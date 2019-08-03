#!/usr/bin/env python2
# coding: UTF-8

# 複数台のロボットを同時に動かすサンプルプログラム

import rospy
from consai2_msgs.msg import ControlTarget, DecodedReferee
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

import math

decoded_referee = DecodedReferee()

def callback_decodedref(msg):
    global decoded_referee
    decoded_referee = msg

class CollectiveController(object):
    def __init__(self):
        self._ID_NUM = 16

        self._is_busy = False
        self._move_forward = True

        self._is_arrived = []
        self._control_target = []

        self._pubs_control_target = []
        self._subs_is_arrived = []

        color = 'blue'
        for robot_id in range(self._ID_NUM):
            self._is_arrived.append(False)
            self._control_target.append(ControlTarget())

            # 末尾に16進数の文字列をつける
            topic_id = hex(robot_id)[2:]

            topic_name = 'consai2_game/control_target_' + color +'_' + topic_id
            pub_control_target = rospy.Publisher(topic_name, ControlTarget,
                    queue_size=1)
            self._pubs_control_target.append(pub_control_target)

            topic_name = 'consai2_control/is_arrived_' + color +'_' + topic_id
            sub_is_arrived = rospy.Subscriber(topic_name, Bool, 
                    self._callback_is_arrived, queue_size=1, callback_args=robot_id)
            self._subs_is_arrived.append(sub_is_arrived)


    def _callback_is_arrived(self, msg, robot_id):
        self._is_arrived[robot_id] = msg.data


    def halt(self):
        # 全てのロボットの制御を切る
        for robot_id in range(self._ID_NUM):
            control_target = ControlTarget()
            control_target.robot_id = robot_id
            control_target.control_enable = False

            self._pubs_control_target[robot_id].publish(control_target)

        # ビジー状態を解除して、halt後に動けるようにする
        self._is_busy = False

    def collective_move(self):
        if self._is_busy is True:
            # 全員が目標位置にたどり着いたかチェック
            if self._are_robots_arrived(6):
                print("Arrived")
                self._is_busy = False

                # 移動方向を変更する
                self._move_forward = not self._move_forward

        if self._is_busy is False:
            # 目標値を送信
            print("Send")
            self._publish_line_positions(self._move_forward)
            self._is_busy = True


    def _publish_line_positions(self, move_forward=True):
        START_X = -2.0
        MARGIN = 0.4
        TARGET_Y = 2.0
        THETA = math.pi * 0.5

        # 横一直線の目標位置を送信する
        for robot_id in range(self._ID_NUM):
            control_target = ControlTarget()
            control_target.robot_id = robot_id
            control_target.control_enable = True

            # ゴール位置を生成
            pose = Pose2D()
            pose.x = START_X + MARGIN*robot_id
            pose.y = TARGET_Y

            if move_forward is False:
                pose.y *= -1.0

            pose.theta = THETA
            control_target.path.append(pose)

            # ゴール地点での速度
            control_target.goal_velocity = Pose2D(0.0, 0.0, 0.0)

            # キック威力 0.0 ~ 1.0
            control_target.kick_power = 0.0
            control_target.chip_enable = False
            control_target.dribble_power = 0.0

            self._pubs_control_target[robot_id].publish(control_target)


    def _are_robots_arrived(self, check_num=3):
        # 指定台数のロボットが目標位置にたどり着けたかをチェック

        counter = 0
        for robot_id in range(self._ID_NUM):
            if self._is_arrived[robot_id] is True:
                counter += 1

        # print "counter: " + str(counter)
        if counter >= check_num:
            return True
        else:
            return False


def main():
    global decoded_referee

    rospy.init_node('collective_control_example')

    sub_decodedref = rospy.Subscriber("referee_wrapper/decoded_referee", 
            DecodedReferee,callback_decodedref,queue_size=1)

    controller = CollectiveController()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():

        if decoded_referee.referee_text == "STOP":
            # 全員動かす
            controller.collective_move()
        else:
            # 全てのロボットを止める
            controller.halt()

        r.sleep()
    

if __name__ == '__main__':
    main()
