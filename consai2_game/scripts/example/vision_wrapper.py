#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import VisionDetections, VisionGeometry


class VisionWrapper(object):
    def __init__(self):
        self._counter = 0
        self._side = rospy.get_param('~side', 'left')
        self._color = rospy.get_param('~color', 'blue')

        # チームサイドの反転
        invert_side = False
        if self._side != 'left':
            invert_side = True

        self._our_robot

    def callback_detections(self, msg):
        pass

    def callback_geometry(self, msg):
        pass


def main():
    rospy.init_node('vision_wrapper')

    wrapper = VisionWrapper()

    sub_detections = rospy.Subscriber(
            'vision_receiver/raw_vision_detections',
            VisionDetections,
            wrapper.callback_detections)

    sub_geometry = rospy.Subscriber(
            'vision_receiver/raw_vision_geometry',
            VisionGeometry,
            wrapper.callback_geometry)

    rospy.spin()



if __name__ == '__main__':
    main()
