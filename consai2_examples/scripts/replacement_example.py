#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import Replacements, ReplaceBall, ReplaceRobot


def robot_places(x, y, plus_x, plus_y, direction, is_yellow):
    replace_robots = []
    for i in range(6):
        robot = ReplaceRobot()
        robot.x = x + plus_x * i
        robot.y = y + plus_y * i
        robot.dir = direction
        robot.id = i
        robot.yellowteam = is_yellow
        robot.turnon = True
        replace_robots.append(robot)

    return replace_robots

def demo(pub):

    replacements = Replacements()

    ball = ReplaceBall()
    ball.x = 0
    ball.y = 0
    ball.vx = 0
    ball.vy = 0
    ball.is_enabled = True

    replacements.ball = ball

    replacements.robots.extend(robot_places(0, 1.0, 0.2, 0.0, 0, False))
    replacements.robots.extend(robot_places(0, -1.0, -0.2, 0.0, 180, True))

    pub.publish(replacements)
    rospy.sleep(0.5)

    replacements.ball.is_enabled = False
    replacements.robots = []

    replacements.robots.extend(robot_places(1.0, 1.0, 0.0, -0.2, 90, False))
    replacements.robots.extend(robot_places(-1.0, -1.0, 0.0, 0.2, -90, True))
    pub.publish(replacements)
    replacements.robots = []
    rospy.sleep(0.5)

    replacements.robots.extend(robot_places(0.2, 0.2, 0.2, 0.2, 45, False))
    replacements.robots.extend(robot_places(-0.2, -0.2, -0.2, -0.2, -135, True))
    pub.publish(replacements)
    replacements.robots = []
    rospy.sleep(0.5)

    replacements.robots.extend(robot_places(0.2, -0.2, 0.2, -0.2, 135, False))
    replacements.robots.extend(robot_places(-0.2, 0.2, -0.2, 0.2, -45, True))
    pub.publish(replacements)
    replacements.robots = []
    rospy.sleep(0.5)

    SLEEP_TIME = 0.1

    replacements.ball.vy = 4.0
    replacements.ball.is_enabled = True
    pub.publish(replacements)
    rospy.sleep(SLEEP_TIME)
    replacements.ball.is_enabled = False

    for i in range(6):
        robot = ReplaceRobot()
        robot.x = 0.5
        robot.y = 0
        robot.dir = 0
        robot.id = i
        robot.yellowteam = False
        robot.turnon = True
        replacements.robots.append(robot)
        pub.publish(replacements)
        replacements.robots = []
        rospy.sleep(SLEEP_TIME)

    replacements.ball.vy = -4.0
    replacements.ball.is_enabled = True
    pub.publish(replacements)
    rospy.sleep(SLEEP_TIME)
    replacements.ball.is_enabled = False
    for i in range(6):
        robot = ReplaceRobot()
        robot.x = -0.5
        robot.y = 0
        robot.dir = 180
        robot.id = i
        robot.yellowteam = True
        robot.turnon = True
        replacements.robots.append(robot)
        pub.publish(replacements)
        replacements.robots = []
        rospy.sleep(SLEEP_TIME)

    for i in range(6):
        robot = ReplaceRobot()
        robot.x = 0
        robot.y = 0
        robot.dir = 0
        robot.id = i
        robot.yellowteam = False
        robot.turnon = True
        replacements.robots.append(robot)
        pub.publish(replacements)
        replacements.robots = []
        rospy.sleep(SLEEP_TIME)

        robot.yellowteam = True
        replacements.robots.append(robot)
        pub.publish(replacements)
        replacements.robots = []
        rospy.sleep(SLEEP_TIME)

    replacements.ball.vy = 0
    replacements.ball.is_enabled = True
    for i in range(6):
        robot = ReplaceRobot()
        robot.x = 0
        robot.y = 0
        robot.dir = 0
        robot.id = i
        robot.yellowteam = False
        robot.turnon = True
        replacements.robots.append(robot)

        robot.yellowteam = True
        replacements.robots.append(robot)

    pub.publish(replacements)


def main():
    rospy.init_node('replacement_example')
    pub = rospy.Publisher('sim_sender/replacements', Replacements, queue_size=1)

    # grSimのロボットとボールを配置する

    rospy.sleep(1)
    
    # demo(pub)

    replacements = Replacements()

    # ボールの配置
    ball = ReplaceBall()
    ball.x = 0
    ball.y = 0
    ball.vx = 0
    ball.vy = 0
    ball.is_enabled = True

    replacements.ball = ball

    # 青ロボットの配置
    for i in range(6):
        robot = ReplaceRobot()
        robot.x = 0.5 + 0.5 * i
        robot.y = 0
        robot.dir = 90
        robot.id = i
        robot.yellowteam = False
        robot.turnon = True
        replacements.robots.append(robot)

    # 黄ロボットの配置
    for i in range(6):
        robot = ReplaceRobot()
        robot.x = -0.5 - 0.5 * i
        robot.y = 0
        robot.dir = -90
        robot.id = i
        robot.yellowteam = True
        robot.turnon = True
        replacements.robots.append(robot)

    pub.publish(replacements)

if __name__ == '__main__':
    main()
