#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D


def main():
    rospy.init_node('control_example')

    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    COLOR = "blue" # 'blue' or 'yellow'
    TARGET_ID = 0 # 0 ~ MAX_ID

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = 'consai2_game/control_target_' + COLOR +'_' + topic_id

    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    print 'control_exmaple start'
    rospy.sleep(3.0)

    # 制御目標値を生成
    control_target = ControlTarget()

    # ロボットID
    control_target.robot_id = TARGET_ID
    # Trueで走行開始
    control_target.control_enable = True
    # ゴール姿勢 x, y, theta
    goal_pose = Pose2D()
    goal_pose.x = 0.0 # meter
    goal_pose.y = 0.0 # meter
    goal_pose.theta = 0.0 # radians
    control_target.goal_pose = goal_pose

    # ゴール地点での速度
    goal_velocity = Pose2D(0.0, 0.0, 0.0)
    control_target.goal_velocity = goal_velocity

    # 経路追従を実行するか
    control_target.use_path = False
    control_target.path = []

    # キック威力 0.0 ~ 1.0
    control_target.kick_power = 0.0
    control_target.chip_enable = False
    control_target.dribble_power = 0.0


    pub.publish(control_target)
    print 'control_exmaple finish'

if __name__ == '__main__':
    main()
