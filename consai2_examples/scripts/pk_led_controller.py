# coding: UTF-8

import copy
import rospy
from consai2_msgs.msg import BallInfo
from consai2_msgs.msg import VisionGeometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import ColorRGBA

import math
import random

class LEDController(object):
    def __init__(self):
        self._pub_led_left  = rospy.Publisher('led_left',  ColorRGBA, queue_size = 1)
        self._pub_led_right = rospy.Publisher('led_right', ColorRGBA, queue_size = 1)

        self._COLOR_ATTACKER_INACTIVE = ColorRGBA(1.0, 0.0, 0.0, 0.0)
        self._COLOR_ATTACKER_ACTIVE = ColorRGBA(0.0, 0.0, 1.0, 0.0)
        self._COLOR_SHOOT = ColorRGBA(1.0, 1.0, 1.0, 0.0)
        self._GOAL_LED_LIGHTING_TIME = 5.0  # seconds

        self._goal_led_height = 0.0
        self._goal_timestamp = 0.0
        self._has_scored = False

    def publish_led_colors(self, attacker_can_move, ball_info,
        field_length, field_width, goal_width,
        kazasu_left, kazasu_right, foot_switch_has_pressed, rostime_now):
        led_left = ColorRGBA()  # r(0 ~ 1), g(0 ~ 1), b(0 ~ 1), LED点灯数(0 ~ 1)
        led_right = ColorRGBA()

        if attacker_can_move is False:
            # アタッカーが動けないときは、kazasuが反応したときに赤色で点灯する
            # 動かせないよ〜というサイン
            led_left = copy.deepcopy(self._COLOR_ATTACKER_INACTIVE)
            led_right = copy.deepcopy(self._COLOR_ATTACKER_INACTIVE)

            if kazasu_left:
                led_left.a = 1.0

            if kazasu_right:
                led_right.a = 1.0
        else:
            # アタッカーが動けるときは、基本青色に点灯する
            # フットスイッチを押すと白色に点灯
            led_left, led_right = self._inplay_leds(
                kazasu_left, kazasu_right, foot_switch_has_pressed)

            # ボールがゴールに入ったかを判定
            if self._ball_is_in_goal(ball_info.pose, field_length, goal_width) \
                and self._has_scored is False:
                # ゴールに入ったときの時間を取得
                self._goal_timestamp = rostime_now.to_sec()
                self._has_scored = True
            
            # ゴール演出
            if self._has_scored:
                led_left, led_right = self._goal_leds()
                # 一定時間経過したらLEDを通常に戻す
                if rostime_now.to_sec() - self._goal_timestamp > self._GOAL_LED_LIGHTING_TIME:
                    self._has_scored = False
        
        self._pub_led_left.publish(led_left)
        self._pub_led_right.publish(led_right)

    def _inplay_leds(self, kazasu_left, kazasu_right, foot_switch_has_pressed):
        # アタッカー移動中のLED点灯
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

        return led_left, led_right

    def _goal_leds(self):
        # ゴール演出
        # ランダムなカラーでLEDを点灯する
        self._goal_led_height += 0.02  # LEDの高さ（点灯数）をインクリメント
        if self._goal_led_height > 1.0:
            self._goal_led_height = 0.0
        random_led = ColorRGBA(random.random(), random.random(), random.random(), self._goal_led_height)

        return random_led, random_led

    def _ball_is_in_goal(self, ball_pose, field_length, goal_width):
        # ボールがゴールに入ったかを判定
        if math.fabs(ball_pose.y) < goal_width * 0.5\
            and ball_pose.x < -field_length * 0.5:
            return True
        else:
            return False
