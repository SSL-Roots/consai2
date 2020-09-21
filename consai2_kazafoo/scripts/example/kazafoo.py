#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

import sys, select, termios, tty
import serial


class KazafooCom(threading.Thread):
    def __init__(self):
        super(KazafooCom, self).__init__()
        self.daemon = True

        self._DEVICE  = rospy.get_param('consai2_description/kazafoo_device', '/dev/ttyACM0')
        self._BAUDRATE   = rospy.get_param('consai2_description/kazafoo_device', 9600)

        self._serial = serial.Serial(self._DEVICE, self._BAUDRATE)

        self.left_value = 0.0
        self.right_value = 0.0

    def run(self):
        while(1):
            data = self._serial.readline()
            try:
                left_value_raw = data.split(',')[0]
                right_value_raw = data.split(',')[1]
            except IndexError:
                continue

            try:
                self.left_value = float(left_value_raw)
            except:
                self.left_value = 0.0
            try:
                self.right_value = float(right_value_raw)
            except:
                self.right_value = 0.0


class KeyboardThread(threading.Thread):
    def __init__(self):
        super(KeyboardThread, self).__init__()
        self.daemon = True

        self.settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        self.kick_flag = False

    def run(self):
        TIMEOUT = 0.1
        OFF_THRESH_SEC = 1.0
        off_count = 0

        while(1):
            key = self.getKey(TIMEOUT)
            if key == 'b':
                off_count = 0
            elif key == '\x03':
                break
            else:
                if off_count <= int(OFF_THRESH_SEC / TIMEOUT):
                    off_count += 1

            if off_count > int(OFF_THRESH_SEC / TIMEOUT):
                self.kick_flag = False
            else:
                self.kick_flag = True


    def getKey(self, key_timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


if __name__=="__main__":
    rospy.init_node('kazafoo_node')

    publisher = rospy.Publisher('joy_kazafoo', Joy, queue_size = 1)
    pub_kazasu_left = rospy.Publisher('kazasu_left', Float32, queue_size = 1)
    pub_kazasu_right = rospy.Publisher('kazasu_right', Float32, queue_size = 1)

    kazafoo_com_thread = KazafooCom()
    kazafoo_com_thread.start()

    keyboard_thread = KeyboardThread()
    keyboard_thread.start()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        joy = Joy()
        joy.buttons = [0.0] * 3
        joy.buttons[0] = 1 if keyboard_thread.kick_flag else 0
        joy.buttons[1] = 1 if kazafoo_com_thread.left_value > 0.5 else 0
        joy.buttons[2] = 1 if kazafoo_com_thread.right_value > 0.5 else 0

        pub_kazasu_left.publish(kazafoo_com_thread.left_value)
        pub_kazasu_right.publish(kazafoo_com_thread.right_value)
        publisher.publish(joy)
        r.sleep()