#!/usr/bin/env python2
# coding: UTF-8

import rospy
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
from consai2_msgs.msg import VisionDetections, VisionGeometry, BallInfo, RobotInfo
import math
import tool

ball_pose = Pose2D()
robot_pose = Pose2D()
target_pose = Pose2D()
xg = -6
yg = 0
xr = -5.5

RobotRadius = 0.09
BallRadius = 0.0215

def path_example(target_id):
    
    # 制御目標値を生成
    control_target = ControlTarget()

    # ロボットID
    control_target.robot_id = target_id
    # Trueで走行開始
    control_target.control_enable = True

    coordinate = Coordinate()
    coordinate._update_approach_to_shoot()

    control_target.path.append(target_pose)

    return control_target

def BallPose(data):
    global ball_pose
    ball_pose = data.pose

def RobotPose(data):
    global robot_pose
    robot_pose = data.pose

def main():
    rospy.init_node('control_example')
    
    MAX_ID = rospy.get_param('consai2_description/max_id', 15)
    COLOR = "blue" # 'blue' or 'yellow'
    TARGET_ID = 1 # 0 ~ MAX_ID

    # 末尾に16進数の文字列をつける
    topic_id = hex(TARGET_ID)[2:]
    topic_name = 'consai2_game/control_target_' + COLOR +'_' + topic_id

    # print sub
    pub = rospy.Publisher(topic_name, ControlTarget, queue_size=1)

    print 'control_exmaple start'
    rospy.sleep(3.0)

    # 制御目標値を生成
    r = rospy.Rate(60)
    while 1:
        # ballの位置を取得する
        sub_ball = rospy.Subscriber('vision_wrapper/ball_info', BallInfo, BallPose)
        # Robotの位置を取得する
        sub_robot = rospy.Subscriber('vision_wrapper/robot_info_blue_1', RobotInfo, RobotPose)

        control_target = path_example(TARGET_ID)
        pub.publish(control_target)
        r.sleep()

    print 'control_exmaple finish'

class Coordinate(object):
    # Coordinateクラスは、フィールド状況をもとに移動目標位置、目標角度を生成する
    # Coordinateクラスには、移動目標の生成方法をsetしなければならない
    # Coordinateクラスのposeが生成された移動目標である

    def __init__(self):
        self.pose = Pose2D() # pos_x, pos_y, thta

        self._base = None # string data
        self._target = None # string data
        self._update_func = None

        # arrival parameters
        self._arrived_position_tolerance = 0.1 # unit:meter
        self._arrived_angle_tolerance = 3.0 * math.pi / 180.0

        # interpose
        self._to_dist = None
        self._from_dist = None

        # approach to shoot
        self._pose_max = Pose2D(-6,0,0)
        self._my_role = None
        self._role_is_lower_side = False
        self._role_pose_hystersis = 0.1
        self._tuning_param_x = 0.3
        self._tuning_param_y = 0.3
        self._tuning_param_pivot_y = 0.1
        self._tuning_angle = 30.0 * math.pi / 180.0  # 0 ~ 90 degree, do not edit 'math.pi / 180.0'

        self._pose_max.x = BallRadius + self._tuning_param_x
        self._pose_max.y = BallRadius + RobotRadius + self._tuning_param_y


        # receive_ball
        self._can_receive_dist = 1.0 # unit:meter
        self._can_receive_hysteresis = 0.3
        self._receiving = False


    def _update_approach_to_shoot(self):
        # Reference to this idea
        # http://wiki.robocup.org/images/f/f9/Small_Size_League_-_RoboCup_2014_-_ETDP_RoboDragons.pdf

        global robot_pose, target_pose, ball_pose

        # ball_pose = WorldModel.get_pose('Ball')
        _target_pose = Pose2D(6,0,0)
        _role_pose = robot_pose

        if _target_pose is None or _role_pose is None:
            return False

        # ボールからターゲットを見た座標系で計算する
        angle_ball_to_target = tool.getAngle(ball_pose, _target_pose)
        trans = tool.Trans(ball_pose, angle_ball_to_target)
        tr_role_pose = trans.transform(_role_pose)

        # tr_role_poseのloser_side判定にヒステリシスをもたせる
        if self._role_is_lower_side == True and \
                tr_role_pose.y > self._role_pose_hystersis:
            self._role_is_lower_side = False

        elif self._role_is_lower_side == False and \
                tr_role_pose.y < - self._role_pose_hystersis:
            self._role_is_lower_side = True

        if self._role_is_lower_side:
            tr_role_pose.y *= -1.0


        tr_approach_pose = Pose2D()
        if tr_role_pose.x > 0:
            # 1.ボールの斜め後ろへ近づく

            # copysign(x,y)でyの符号に合わせたxを取得できる
            tr_approach_pose = Pose2D(
                    -self._pose_max.x,
                    math.copysign(self._pose_max.y, tr_role_pose.y), 
                    0)

        else:
            # ボール裏へ回るためのピボットを生成
            pivot_pose = Pose2D(0, self._tuning_param_pivot_y, 0)
            angle_pivot_to_role = tool.getAngle(pivot_pose,tr_role_pose)

            limit_angle = self._tuning_angle + math.pi * 0.5

            if tr_role_pose.y > self._tuning_param_pivot_y and \
                    angle_pivot_to_role < limit_angle:
                # 2.ボール後ろへ回りこむ
            
                diff_angle = tool.normalize(limit_angle - angle_pivot_to_role)
                decrease_coef = diff_angle / self._tuning_angle
            
                tr_approach_pose = Pose2D(
                        -self._pose_max.x,
                        self._pose_max.y * decrease_coef, 
                        0)
            
            else:
                # 3.ボールに向かう

                diff_angle = tool.normalize(angle_pivot_to_role - limit_angle)
                approach_coef = diff_angle / (math.pi * 0.5 - self._tuning_angle)
            
                if approach_coef > 1.0:
                    approach_coef = 1.0

                pos_x = approach_coef * (2.0 * BallRadius 
                        - self._tuning_param_x) + self._tuning_param_x
            
                tr_approach_pose = Pose2D(-pos_x, 0, 0)

        # 上下反転していたapproach_poseを元に戻す
        if self._role_is_lower_side:
            tr_approach_pose.y *= -1.0

        self.pose = trans.invertedTransform(tr_approach_pose)
        self.pose.theta = angle_ball_to_target

        target_pose = self.pose
        
        return True

if __name__ == '__main__':
    main()
