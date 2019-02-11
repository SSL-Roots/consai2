#!/usr/bin/env python2
# coding: UTF-8

import rospy, unittest, rostest
import rosnode
import time

class TestVisionReceiver(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/vision_receiver', nodes, 'node does not exist')

if __name__ == '__main__':
    time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
    rospy.init_node('test_vision_receiver')
    rostest.rosrun('consai2_receiver', 'test_vision_receiver', TestVisionReceiver)
