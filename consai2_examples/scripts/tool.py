 
# -*- coding: utf-8 -*-
import math
import cmath
import tf
import numpy

from geometry_msgs.msg import Pose2D


def normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def getAngle(fromPose, toPose):
    diffPose = Pose2D()

    diffPose.x = toPose.x - fromPose.x
    diffPose.y = toPose.y - fromPose.y

    return math.atan2(diffPose.y, diffPose.x)



class Trans():
    def __init__(self, center , theta):

        normalizedTheta = normalize(theta)
        self._c_center = center.x + center.y * 1.0j
        self._c_rotate = cmath.rect(1.0,normalizedTheta) 
        self._c_angle = normalizedTheta

    def transform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = (c_point - self._c_center) * numpy.conj(self._c_rotate)

        output = Pose2D()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def invertedTransform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = c_point * self._c_rotate + self._c_center

        output = Pose2D()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def transformAngle(self, angle):
        return normalize(angle - self._c_angle)

    def invertedTransformAngle(self, angle):
        return normalize(angle + self._c_angle)

