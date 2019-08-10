#!/usr/bin/env python2
# coding: UTF-8

import rospy
import multicast
import math

from consai2_msgs.msg import VisionGeometry, FieldLineSegment, FieldCircularArc
from consai2_msgs.msg import VisionDetections, DetectionFrame, DetectionBall, DetectionRobot
from consai2_receiver_proto import messages_robocup_ssl_wrapper_pb2 as ssl_wrapper



class FormatConverter(object):
    def __init__(self, invert_side):
        self._invert_side = invert_side

        self._vision_detections = VisionDetections()

        self._TO_METER = 0.001

    def append_frame(self, packet_detection):
        detection_frame = DetectionFrame()
        detection_frame.t_capture = packet_detection.t_capture
        detection_frame.t_sent    = packet_detection.t_sent
        detection_frame.camera_id = packet_detection.camera_id

        # ball
        for ball in packet_detection.balls:
            detection_frame.balls.append(self._convert_to_ball_topic(ball))

        # blue robot
        for robot in packet_detection.robots_blue:
            detection_frame.robots_blue.append(self._convert_to_robot_topic(robot))

        # yellow robot
        for robot in packet_detection.robots_yellow:
            detection_frame.robots_yellow.append(self._convert_to_robot_topic(robot))

        self._vision_detections.frames.append(detection_frame)


    def frame_clear(self):
        self._vision_detections.frames = []

    def get_frame(self):
        return self._vision_detections


    def _convert_to_ball_topic(self, raw_ball):
        detection_ball = DetectionBall()

        detection_ball.pose.x = raw_ball.x * self._TO_METER
        detection_ball.pose.y = raw_ball.y * self._TO_METER

        if self._invert_side:
            detection_ball.pose.x = -detection_ball.pose.x
            detection_ball.pose.y = -detection_ball.pose.y

        return detection_ball

    def _convert_to_robot_topic(self, raw_robot):
        detection_robot = DetectionRobot()

        detection_robot.robot_id   = raw_robot.robot_id
        detection_robot.pose.x = raw_robot.x * self._TO_METER
        detection_robot.pose.y = raw_robot.y * self._TO_METER
        detection_robot.pose.theta  = raw_robot.orientation

        if self._invert_side:
            detection_robot.pose.x = -detection_robot.pose.x
            detection_robot.pose.y = -detection_robot.pose.y
            detection_robot.pose.theta += math.pi

        return detection_robot


class VisionReceiver(object):
    def __init__(self):
        self._HOST = rospy.get_param('consai2_description/vision_addr', '224.5.23.2')
        self._PORT = rospy.get_param('consai2_description/vision_port', 10006)
        self._SIDE = rospy.get_param('consai2_description/our_side', 'left')
        self._sock = multicast.Multicast(self._HOST, self._PORT)

        self._pub_geometry = rospy.Publisher(
                '~raw_vision_geometry', VisionGeometry, queue_size=1)
        self._pub_detection = rospy.Publisher(
                '~raw_vision_detections', VisionDetections, queue_size=1)

        # チームサイドの反転
        # FIXME: チームサイドの反転はより上位の vision_wrapperにやらせる
        invert_side = False
        if self._SIDE != 'left':
            invert_side = True

        self._converter = FormatConverter(invert_side)

        self._TO_METER = 0.001


    def receive(self):
        BUF_LENGTH = 2048

        buf = ""
        self._converter.frame_clear()

        # 複数のカメラからデータを受け取るため、whileループで全て受け取る
        while buf is not False:
            buf = self._sock.recv(BUF_LENGTH)
            if buf:
                # do something
                packet = ssl_wrapper.SSL_WrapperPacket()
                packet.ParseFromString(buf)

                if packet.HasField('detection'):
                    self._converter.append_frame(packet.detection)

                if packet.HasField('geometry'):
                    self._publish_geometry(packet.geometry.field)

        self._publish_detection()


    def _publish_detection(self):
        vision_detection = self._converter.get_frame()

        if vision_detection.frames:
            vision_detection.header.stamp = rospy.Time.now()
            self._pub_detection.publish(vision_detection)

    def _publish_geometry(self, packet_field):
        # Visionデータからフィールド情報を取り出しトピックとして出力する

        geometry = VisionGeometry()

        geometry.field_length = packet_field.field_length * self._TO_METER
        geometry.field_width = packet_field.field_width * self._TO_METER
        geometry.goal_width = packet_field.goal_width * self._TO_METER
        geometry.goal_depth = packet_field.goal_depth * self._TO_METER
        geometry.boundary_width = packet_field.boundary_width * self._TO_METER

        for line in packet_field.field_lines:
            line_segment = FieldLineSegment()

            line_segment.name = line.name
            line_segment.p1_x = line.p1.x * self._TO_METER
            line_segment.p1_y = line.p1.y * self._TO_METER
            line_segment.p2_x = line.p2.x * self._TO_METER
            line_segment.p2_y = line.p2.y * self._TO_METER
            line_segment.thickness = line.thickness * self._TO_METER
            geometry.field_lines.append(line_segment)
        
        for arc in packet_field.field_arcs:
            circular_arc = FieldCircularArc()

            circular_arc.name = arc.name
            circular_arc.center_x = arc.center.x * self._TO_METER
            circular_arc.center_y = arc.center.y * self._TO_METER
            circular_arc.radius = arc.radius * self._TO_METER
            circular_arc.a1 = arc.a1
            circular_arc.a2 = arc.a2
            circular_arc.thickness = arc.thickness * self._TO_METER
            geometry.field_arcs.append(circular_arc)

        self._pub_geometry.publish(geometry)


if __name__ == '__main__':
    rospy.init_node('vision_receiver')

    receiver = VisionReceiver()

    r   = rospy.Rate(60)
    while not rospy.is_shutdown():
        receiver.receive()

        r.sleep()
