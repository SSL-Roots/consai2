#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D


def path_example(target_id, publisher):
    # 制御目標値を生成
    control_target = ControlTarget()

    # ロボットID
    control_target.robot_id = target_id
    # Trueで走行開始
    control_target.control_enable = True

    # 経由位置1
    pose = Pose2D()
    pose.x = 1.0 # meter
    pose.y = 1.0 # meter
    pose.theta = 0.0 # radians
    control_target.path.append(pose)

    # 経由位置2
    pose = Pose2D()
    pose.x = -1.0 # meter
    pose.y = 1.0 # meter
    pose.theta = 0.0 # radians
    control_target.path.append(pose)

    # 経由位置3
    pose = Pose2D()
    pose.x = -1.0 # meter
    pose.y = -1.0 # meter
    pose.theta = 0.0 # radians
    control_target.path.append(pose)

    # ゴール位置
    pose = Pose2D()
    pose.x = 0.0 # meter
    pose.y = 0.0 # meter
    pose.theta = 0.0 # radians
    control_target.path.append(pose)

    # ゴール地点での速度
    goal_velocity = Pose2D(0.0, 0.0, 0.0)
    control_target.goal_velocity = goal_velocity

    # キック威力 0.0 ~ 1.0
    control_target.kick_power = 0.0
    control_target.chip_enable = False
    control_target.dribble_power = 0.0

    publisher.publish(control_target)

def publish_velocity(control_target, goal_velocity, publisher, message, sleep_time):
    print message
    control_target.goal_velocity = goal_velocity
    print control_target.goal_velocity
    publisher.publish(control_target)
    rospy.sleep(sleep_time)

def velocity_control_example(target_id, publisher):
    control_target = ControlTarget()

    # ロボットID
    control_target.robot_id = target_id
    # キック・ドリブルをFalse
    control_target.kick_power = 0.0
    control_target.chip_enable = False
    control_target.dribble_power = 0.0

    # Trueで走行開始
    control_target.control_enable = True

    STOP_TIME = 2.0
    MOVE_TIME = 2.0
    
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    publish_velocity(control_target, Pose2D(1, 0, 0), publisher, "Move:", MOVE_TIME)
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    publish_velocity(control_target, Pose2D(-1, 0, 0), publisher, "Move:", MOVE_TIME)
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    publish_velocity(control_target, Pose2D(0, 1, 0), publisher, "Move:", MOVE_TIME)
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    publish_velocity(control_target, Pose2D(0, -1, 0), publisher, "Move:", MOVE_TIME)
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    publish_velocity(control_target, Pose2D(0, 0, math.pi), publisher, "Move:", MOVE_TIME)
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    publish_velocity(control_target, Pose2D(0, 0, -math.pi), publisher, "Move:", MOVE_TIME)
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Stop:", STOP_TIME)

    control_target.control_enable = False
    publish_velocity(control_target, Pose2D(0, 0, 0), publisher, "Finish:", STOP_TIME)

def main():
    rospy.init_node('control_example')

    VELOCITY_CONTROL = rospy.get_param('~velocity_control', False)
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    TARGET_ID = rospy.get_param('~id', 0)
    COLOR = rospy.get_param('~color', 'blue')

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = 'consai2_game/control_target_' + COLOR +'_' + topic_id

    pub_control_target = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    print 'control_exmaple start'
    rospy.sleep(3.0)

    print VELOCITY_CONTROL
    # 制御目標値を生成
    if VELOCITY_CONTROL is True:
        print 'Velocity control'
        velocity_control_example(TARGET_ID, pub_control_target)
    else:
        print 'Path tracking'
        path_example(TARGET_ID, pub_control_target)

    print 'control_exmaple finish'

if __name__ == '__main__':
    main()


