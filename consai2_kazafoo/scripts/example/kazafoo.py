#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from sensor_msgs.msg import Joy

import sys, select, termios, tty
import serial

import time

moveBindings = {
        'b':(0,0,-1,0),
    }


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
            left_value_raw = data.split(',')[0]
            right_value_raw = data.split(',')[1]

            try:
                self.left_value = float(left_value_raw)
            except:
                self.left_value = 0.0
            try:
                self.right_value = float(right_value_raw)
            except:
                self.right_value = 0.0


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('joy', Joy, queue_size = 1)
        self.kick_flag = False
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, kick_flag):
        self.condition.acquire()
        self.kick_flag = kick_flag
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(False)
        self.join()

    def run(self):
        joy = Joy()
        joy.buttons = [0] * 3
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into joy message.
            joy.buttons[0] = 1.0 if self.kick_flag else 0.0

            self.condition.release()

            # Publish.
            self.publisher.publish(joy)

        # Publish stop message when thread exits.
        joy.buttons[0] = 0.0
        self.publisher.publish(joy)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    # settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('kazafoo_node')

    # speed = rospy.get_param("~speed", 0.5)
    # turn = rospy.get_param("~turn", 1.0)
    # repeat = rospy.get_param("~repeat_rate", 0.0)
    # key_timeout = rospy.get_param("~key_timeout", 0.0)
    # if key_timeout == 0.0:
    #     key_timeout = 0.1

    # pub_thread = PublishThread(repeat)

    kazafoo_com_thread = KazafooCom()
    kazafoo_com_thread.start()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("looping")
        print(kazafoo_com_thread.left_value)
        print(kazafoo_com_thread.right_value)
        r.sleep()

    kick_flag = False
    # try:
    #     pub_thread.update(kick_flag)

    #     while(1):
    #         key = getKey(key_timeout)
    #         if key == 'b':
    #             kick_flag = True
    #         else:
    #             kick_flag = False
    #             if (key == '\x03'):
    #                 break
 
    #         pub_thread.update(kick_flag)

    # except Exception as e:
    #     print(e)

    # finally:
    #     pub_thread.stop()

    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)