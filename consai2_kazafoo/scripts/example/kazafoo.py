#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA

import sys, select, termios, tty
import serial

NUM_OF_LED = 30

led_data = {
    "left": {
        "r": 0.0,
        "g": 0.0,
        "b": 0.0,
        "n": 0.0,
    },
    "right": {
        "r": 0.0,
        "g": 0.0,
        "b": 0.0,
        "n": 0.0,
    }
}

def callback_led_left(data):
    global led_data
    led_data["left"]["r"] = data.r
    led_data["left"]["g"] = data.g
    led_data["left"]["b"] = data.b
    led_data["left"]["n"] = data.a


def callback_led_right(data):
    global led_data
    led_data["right"]["r"] = data.r
    led_data["right"]["g"] = data.g
    led_data["right"]["b"] = data.b
    led_data["right"]["n"] = data.a


class KazafooCom:
    def __init__(self):
        self._DEVICE  = rospy.get_param('consai2_description/kazafoo_device', '/dev/ttyACM0')
        self._BAUDRATE   = rospy.get_param('consai2_description/kazafoo_device', 9600)

        self._serial = serial.Serial(self._DEVICE, self._BAUDRATE, timeout=1.0)

    def getSensorValues(self):
        self._serial.write('GS\n'.encode('ascii'))
        data = self._serial.readline()

        try:
            left_value_raw = data.split(',')[0]
            right_value_raw = data.split(',')[1]
        except IndexError:
            return (0.0, 0.0)

        try:
            left_value = float(left_value_raw)
        except:
            left_value = 0.0
        try:
            right_value = float(right_value_raw)
        except:
            right_value = 0.0

        return (left_value, right_value)

    def setLedLeft(self, r, g, b, n):
        self._setLed(True, r, g, b, n)

    def setLedRight(self, r, g, b, n):
        self._setLed(False, r, g, b, n)

    def _setLed(self, is_left, r, g, b, n):
        r_raw = str(int(min(99         * r, 99)        )).zfill(2)
        g_raw = str(int(min(99         * g, 99)        )).zfill(2)
        b_raw = str(int(min(99         * b, 99)        )).zfill(2) 
        n_raw = str(int(min(NUM_OF_LED * n, NUM_OF_LED))).zfill(2)
        
        header = 'LL' if is_left else 'LR'

        self._serial.write((header + r_raw + g_raw + b_raw + n_raw + '\n').encode('ascii'))


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

    sub_led_left  = rospy.Subscriber('led_left',  ColorRGBA, callback_led_left)
    sub_led_right = rospy.Subscriber('led_right', ColorRGBA, callback_led_right)

    kazafoo_com = KazafooCom()

    keyboard_thread = KeyboardThread()
    keyboard_thread.start()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        # set led
        kazafoo_com.setLedLeft (led_data['left'] ['r'], led_data['left'] ['g'], led_data['left'] ['b'], led_data['left'] ['n'])
        kazafoo_com.setLedRight(led_data['right']['r'], led_data['right']['g'], led_data['right']['b'], led_data['right']['n'])

        # get sensor values
        (sensor_left, sensor_right) = kazafoo_com.getSensorValues()

        # publish sensor values
        joy = Joy()
        joy.buttons = [0.0] * 3
        joy.buttons[0] = 1 if keyboard_thread.kick_flag else 0
        joy.buttons[1] = 1 if sensor_left > 0.5 else 0
        joy.buttons[2] = 1 if sensor_right > 0.5 else 0
        joy.axes = [0.0] * 3
        joy.axes[1] = sensor_left
        joy.axes[2] = sensor_right
        publisher.publish(joy)

        r.sleep()
