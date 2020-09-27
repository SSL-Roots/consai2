#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA

def callback_kazafoo_joy(data):
    led_left  = ColorRGBA()
    led_right = ColorRGBA()
    
    if data.buttons[0] == 0.0:
        led_left.r = 0.0
        led_left.g = 1.0
        led_left.b = 0.0
        led_left.a = data.axes[1]

        led_right.r = 0.0
        led_right.g = 1.0
        led_right.b = 0.0
        led_right.a = data.axes[2]
    else:
        # kick! 
        led_left.r = 1.0
        led_left.g = 0.0
        led_left.b = 0.0
        led_left.a = 1.0

        led_right.r = 1.0
        led_right.g = 0.0
        led_right.b = 0.0
        led_right.a = 1.0

    pub_led_left.publish(led_left)
    pub_led_right.publish(led_right)


if __name__=="__main__":
    rospy.init_node('kazafoo_sample_client')

    pub_led_left  = rospy.Publisher('led_left',  ColorRGBA, queue_size = 1)
    pub_led_right = rospy.Publisher('led_right', ColorRGBA, queue_size = 1)
    sub_kazafoo_joy = rospy.Subscriber('joy_kazafoo',  Joy, callback_kazafoo_joy)

    rospy.spin()