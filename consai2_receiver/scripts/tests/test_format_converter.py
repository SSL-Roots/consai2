#!/usr/bin/env python2
# coding: UTF-8

import sys, os
import unittest
import math

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from vision_receiver import FormatConverter


class TestFormatConverter(unittest.TestCase):
    def setUp(self):
        pass

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('consai2_receiver', 'test_format_converter', TestFormatConverter)
