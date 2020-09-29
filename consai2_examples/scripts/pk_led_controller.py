# coding: UTF-8

import copy
import rospy
from consai2_msgs.msg import BallInfo
from consai2_msgs.msg import VisionGeometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import ColorRGBA

import math

class LEDController(object):
    def __init__(self):
        self._pub_led_left  = rospy.Publisher('led_left',  ColorRGBA, queue_size = 1)
        self._pub_led_right = rospy.Publisher('led_right', ColorRGBA, queue_size = 1)

        self._COLOR_ATTACKER_INACTIVE = ColorRGBA(1.0, 0.0, 0.0, 0.0)
        self._COLOR_ATTACKER_ACTIVE = ColorRGBA(0.0, 0.0, 1.0, 0.0)
        self._COLOR_SHOOT = ColorRGBA(1.0, 1.0, 1.0, 0.0)

    def publish_led_colors(self, attacker_can_move, ball_info,
        field_length, field_width, goal_width,
        kazasu_left, kazasu_right, foot_switch_has_pressed):

        led_left = ColorRGBA()
        led_right = ColorRGBA()

        if attacker_can_move is False:
            led_left = copy.deepcopy(self._COLOR_ATTACKER_INACTIVE)
            led_right = copy.deepcopy(self._COLOR_ATTACKER_INACTIVE)

            if kazasu_left:
                led_left.a = 1.0

            if kazasu_right:
                led_right.a = 1.0
        else:
            led_left = copy.deepcopy(self._COLOR_ATTACKER_ACTIVE)
            led_right = copy.deepcopy(self._COLOR_ATTACKER_ACTIVE)

            if kazasu_left:
                led_left.a = 1.0

            if kazasu_right:
                led_right.a = 1.0

            if foot_switch_has_pressed:
                led_left = copy.deepcopy(self._COLOR_SHOOT)
                led_right = copy.deepcopy(self._COLOR_SHOOT)
                led_left.a = led_right.a = 1.0
        
        self._pub_led_left.publish(led_left)
        self._pub_led_right.publish(led_right)

