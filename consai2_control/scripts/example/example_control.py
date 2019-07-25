#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
import serial

from consai2_msgs.msg import ControlTarget, RobotCommand, RobotCommands, RobotInfo
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool


def distance_2_poses(pose1, pose2):
    # 2点間の距離を取る
    # pose.theta は使用しない

    diff_pose = Pose2D()

    diff_pose.x = pose1.x - pose2.x
    diff_pose.y = pose1.y - pose2.y

    return math.hypot(diff_pose.x, diff_pose.y)

def angle_normalize(angle):
    # 角度をpi  ~ -piの範囲に変換する
    while angle > math.pi:
        angle -= 2*math.pi

    while angle < -math.pi:
        angle += 2*math.pi

    return angle


class Controller(object):
    def __init__(self):
        QUEUE_SIZE = 10
        self._COLORS = ['blue', 'yellow']

        self._MAX_VELOCITY = 4.0 # m/s
        self._MAX_ANGLE_VELOCITY = 2.0 * math.pi # rad/s
        self._MAX_ACCELERATION = 1.0 / 60.0 # m/s^2 / frame
        self._MAX_ANGLE_ACCELERATION = 1.0 * math.pi / 60.0 # rad/s^2 / frame
        self._POSE_P_GAIN = 2.0

        self._MAX_ID = rospy.get_param('consai2_description/max_id', 15)

        # フィールド座標系の制御速度
        # PID制御のため、前回の制御速度を保存する
        self._control_velocity = {'blue':[],'yellow':[]}
        for color in self._COLORS:
            for robot_id in range(self._MAX_ID +1):
                self._control_velocity[color].append(Pose2D())

        # 経路追従のためのインデックス
        self._path_index = {'blue':[],'yellow':[]}

        self._robot_info = {'blue':[],'yellow':[]}
        self._subs_robot_info = {'blue':[],'yellow':[]}

        self._control_target = {'blue':[],'yellow':[]}
        self._subs_control_target = {'blue':[],'yellow':[]}

        self._pubs_is_arrived = {'blue':[],'yellow':[]}

        for color in self._COLORS:
            for robot_id in range(self._MAX_ID +1):
                self._robot_info[color].append(RobotInfo())
                self._control_target[color].append(ControlTarget())
                self._path_index[color].append(0)

                # 末尾に16進数の文字列をつける
                topic_id = hex(robot_id)[2:]

                topic_name = 'vision_wrapper/robot_info_' + color +'_' + topic_id
                sub_robot_info = rospy.Subscriber(topic_name, RobotInfo, 
                    self._callback_robot_info, queue_size=QUEUE_SIZE, callback_args=color)
                self._subs_robot_info[color].append(sub_robot_info)

                topic_name = 'consai2_game/control_target_' + color +'_' + topic_id
                sub_control_target = rospy.Subscriber(topic_name, ControlTarget,
                    self._callback_control_target, queue_size=QUEUE_SIZE, callback_args=color)
                self._subs_control_target[color].append(sub_control_target)

                topic_name = 'consai2_control/is_arrived_' + color +'_' + topic_id
                pub_is_arrived = rospy.Publisher(topic_name, Bool, queue_size=QUEUE_SIZE)
                self._pubs_is_arrived[color].append(pub_is_arrived)

        self._pub_commands = rospy.Publisher('consai2_control/robot_commands', 
                RobotCommands, queue_size=QUEUE_SIZE)


    def _callback_robot_info(self, msg, color):
        self._robot_info[color][msg.robot_id] = msg

    def _callback_control_target(self, msg, color):
        self._control_target[color][msg.robot_id] = msg
        # 経路追従を初期化する
        self._path_index[color][msg.robot_id] = 0 


    def _control_update(self, color, robot_id):
        # ロボットの走行制御を更新する
        # ControlTargetを受け取ってない場合は停止司令を生成する

        command = RobotCommand()
        command.robot_id = robot_id

        control_target = self._control_target[color][robot_id]

        # 制御目標が有効であれば
        if control_target.control_enable:
            # 経路がセットされていれば
            if len(control_target.path) != 0:
                command, arrived  = self._path_tracking(color, robot_id, control_target.path)
                # ゴールに到着したら
                # キックやドリブルの制御をenableにする
                if arrived:
                    command.kick_power = control_target.kick_power
                    command.chip_enable = control_target.chip_enable
                    command.dribble_power = control_target.dribble_power
                    # 到達したことをpublish
                    self._pubs_is_arrived[color][robot_id].publish(True)
                else:
                    # 到達してないことをpublish
                    self._pubs_is_arrived[color][robot_id].publish(False)
        else:
            # 保存していた制御速度をリセットする
            self._reset_control_velocity(color, robot_id)

        return command


    def _path_tracking(self, color, robot_id, path):
        ARRIVED_THRESH = 0.2 # meters

        arrived = False
        command = RobotCommand()
        command.robot_id = robot_id

        path_index = self._path_index[color][robot_id]

        if len(path) == 0 :
            # パスがセットされてない場合
            # 何もせず初期値のcommandを返す
            pass
        elif path_index == len(path):
            # ゴールまで到達した場合
            # 最後のposeに移動し続ける
            pose = path[-1]
            command = self._move_to_pose(color, robot_id, pose)
            arrived = True
        else:
            # 経路追従　
            pose = path[path_index]
            command = self._move_to_pose(color, robot_id, pose)

            # poseに近づいたか判定する
            robot_pose = self._robot_info[color][robot_id].pose

            if distance_2_poses(pose, robot_pose) < ARRIVED_THRESH:
                # 近づいたら次の経由位置へ向かう
                self._path_index[color][robot_id] += 1

        return command, arrived


    def _move_to_pose(self, color, robot_id, goal_pose):
        command = RobotCommand()
        command.robot_id = robot_id
        
        robot_info = self._robot_info[color][robot_id]
        current_control_velocity = self._control_velocity[color][robot_id]

        # 制御速度を計算
        # self._control_velocityにも同値を保存する
        control_velocity = self._pid_pose_control(
                color, robot_id, robot_info.pose, goal_pose,
                current_control_velocity)
        # 速度方向をロボット座標系に変換
        theta = robot_info.pose.theta
        command.vel_surge = math.cos(theta)*control_velocity.x + math.sin(theta)*control_velocity.y
        command.vel_sway = -math.sin(theta)*control_velocity.x + math.cos(theta)*control_velocity.y
        command.vel_angular = control_velocity.theta

        return command
    
    def _pid_pose_control(self, color, robot_id, robot_pose, goal_pose, current_control_velocity):
        # 現在姿勢と目標姿勢の差分から、field座標系での動作速度を求める
        diff_pose = Pose2D()
        target_velocity = Pose2D()

        diff_pose.x = goal_pose.x - robot_pose.x
        diff_pose.y = goal_pose.y - robot_pose.y
        diff_pose.theta = angle_normalize(goal_pose.theta - robot_pose.theta)

        # 目標動作速度を求める
        target_velocity.x = self._POSE_P_GAIN * diff_pose.x
        target_velocity.y = self._POSE_P_GAIN * diff_pose.y
        target_velocity.theta = self._POSE_P_GAIN * diff_pose.theta

        # x方向の加速度制限
        current_control_velocity.x = self._acceleration_limit(
                target_velocity.x, current_control_velocity.x,
                self._MAX_ACCELERATION)
        # y方向の加速度制限
        current_control_velocity.y = self._acceleration_limit(
                target_velocity.y, current_control_velocity.y,
                self._MAX_ACCELERATION)
        # thetaの加速度制限
        current_control_velocity.theta = self._acceleration_limit(
                target_velocity.theta, current_control_velocity.theta,
                self._MAX_ANGLE_ACCELERATION, is_angle=True)

        # x方向の速度制限
        current_control_velocity.x = self._velocity_limit(
                current_control_velocity.x, self._MAX_VELOCITY)
        # y方向の速度制限
        current_control_velocity.y = self._velocity_limit(
                current_control_velocity.y, self._MAX_VELOCITY)
        # theta方向の速度制限
        current_control_velocity.theta = self._velocity_limit(
                current_control_velocity.theta, self._MAX_ANGLE_VELOCITY)

        return current_control_velocity

    
    def _acceleration_limit(self, target_velocity, current_velocity, limit_value, is_angle=False):
        # 加速度制限をかけて目標速度を返す
        diff_velocity = target_velocity - current_velocity

        if is_angle:
            diff_velocity = angle_normalize(diff_velocity)

        if math.fabs(diff_velocity) > limit_value:
            target_velocity = current_velocity + math.copysign(limit_value, diff_velocity)

            if is_angle:
                target_velocity = angle_normalize(target_velocity)

        return target_velocity

    def _velocity_limit(self, target_velocity, limit_value):
        # 速度制限をかけて目標速度を返す

        if target_velocity > limit_value:
            target_velocity = limit_value

        elif target_velocity < -limit_value:
            target_velocity = -limit_value

        return target_velocity

    
    def _reset_control_velocity(self, color, robot_id):
        self._control_velocity[color][robot_id] = Pose2D()


    def update(self):
        # consai vs consaiのために、blue, yellow両方のロボットを制御する
        for color in self._COLORS:
            robot_commands = RobotCommands()
            robot_commands.header.stamp = rospy.Time.now()
            if color == 'blue':
                robot_commands.is_yellow = False
            else:
                robot_commands.is_yellow = True

            for robot_info in self._robot_info[color]:
                # フィールド上にいるロボットのみを動かす
                if robot_info.disappeared is False:
                    # 動作司令を更新
                    command = self._control_update(color, robot_info.robot_id)
                    robot_commands.commands.append(command)

            self._pub_commands.publish(robot_commands)


def main():
    rospy.init_node('consai2_control')

    controller = Controller()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        controller.update()
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass
