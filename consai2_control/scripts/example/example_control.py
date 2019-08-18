#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
import copy

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

class PIDGain(object):
    def __init__(self, KP, KI ,KD):
        self._KP = KP
        self._KI = KI
        self._KD = KD

    def KP(self):
        return self._KP
    def KI(self):
        return self._KI
    def KD(self):
        return self._KD

class PIDController(object):
    class inner_PIDController(object):
        def __init__(self, pid_gain):
            self._pid_gain = pid_gain

            self._error1 = 0
            self._error2 = 0
            self._output = 0

        def update(self, error, disable_integrator_input=False):
            delta_output = self._pid_gain.KP() * (error - self._error1)
            if disable_integrator_input == False:
                delta_output += self._pid_gain.KI() * (error)
            delta_output += self._pid_gain.KD() * (error - 2*self._error1 + self._error2)

            self._output += delta_output

            # 誤差の更新
            self._error2 = self._error1
            self._error1 = error
            return self._output

    def __init__(self, pid_gain_x, pid_gain_y, pid_gain_theta):
        self._pid_controller = {
                "x": self.inner_PIDController(pid_gain_x),
                "y": self.inner_PIDController(pid_gain_y),
                "theta": self.inner_PIDController(pid_gain_theta)
                }

        self._output = Pose2D()


    def update(self, error_pose, disable_integrator_input_x, disable_integrator_input_y, disable_integrator_input_theta):
        self._output.x = self._pid_controller["x"].update(error_pose.x, disable_integrator_input_x)
        self._output.y = self._pid_controller["y"].update(error_pose.y, disable_integrator_input_y)
        self._output.theta = self._pid_controller["theta"].update(error_pose.theta, disable_integrator_input_theta)

        return self._output



class Controller(object):
    def __init__(self):
        QUEUE_SIZE = 10
        self._COLORS = ['blue', 'yellow']

        self._MAX_VELOCITY = 2.0 # m/s
        self._MAX_ANGLE_VELOCITY = 2.0 * math.pi # rad/s
        self._MAX_ACCELERATION = 2.0 / 60.0 # m/s^2 / frame
        self._MAX_ANGLE_ACCELERATION = 2.0 * math.pi / 60.0 # rad/s^2 / frame
        self._ARRIVED_THRESH = 0.1 # meters 目標位置に到着したかどうかのしきい値
        self._APPROACH_THRESH = 0.5 # meters 経由位置に近づいたかどうかのしきい値
        self._PID_GAIN = { # 位置制御のPIDゲイン
                "x":PIDGain(1.475, 0.0, 21.0),
                "y":PIDGain(1.475, 0.0, 21.0),
                "theta":PIDGain(1.0, 0.0, 0.0)
                }

        self._MAX_ID = rospy.get_param('consai2_description/max_id', 15)
        self._disable_integrater_input_x = False
        self._disable_integrater_input_y = False
        self._disable_integrater_input_theta = False

        # フィールド座標系の制御速度
        # PID制御のため、前回の制御速度を保存する
        self._control_velocity = {'blue':[],'yellow':[]}
        self._pid_controller = {'blue':[], 'yellow':[]}
        for color in self._COLORS:
            for robot_id in range(self._MAX_ID +1):
                self._control_velocity[color].append(Pose2D())
                self._pid_controller[color].append(
                        PIDController(
                            self._PID_GAIN["x"], 
                            self._PID_GAIN["y"], 
                            self._PID_GAIN["theta"]))

        # 経路追従のためのインデックス
        self._path_index = {'blue':[],'yellow':[]}

        self._robot_info = {'blue':[],'yellow':[]}
        self._subs_robot_info = {'blue':[],'yellow':[]}

        self._control_target = {'blue':[],'yellow':[]}
        self._subs_control_target = {'blue':[],'yellow':[]}

        self._pubs_is_arrived = {'blue':[],'yellow':[]}

        self._pubs_command_velocity = {'blue':[],'yellow':[]}

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

                topic_name = 'consai2_control/command_velocity_' + color + '_' + topic_id
                pub_command_velocity = rospy.Publisher(topic_name, Pose2D, queue_size=QUEUE_SIZE)
                self._pubs_command_velocity[color].append(pub_command_velocity)

        self._pub_commands = rospy.Publisher('consai2_control/robot_commands', 
                RobotCommands, queue_size=QUEUE_SIZE)


    def _callback_robot_info(self, msg, color):
        self._robot_info[color][msg.robot_id] = msg

    def _callback_control_target(self, msg, color):
        self._control_target[color][msg.robot_id] = msg
        # 経路追従を初期化する
        self._path_index[color][msg.robot_id] = 0 


    def _make_command(self, color, robot_id):
        # ロボットの動作司令を生成する
        # ControlTargetを受け取ってない場合は停止司令を生成する

        command = RobotCommand()
        command.robot_id = robot_id

        control_target = self._control_target[color][robot_id]

        # 経路がセットされていれば
        if len(control_target.path) != 0:
            command, arrived  = self._path_tracking(color, robot_id, control_target.path)

            
            if arrived:
                # 到達したことをpublish
                self._pubs_is_arrived[color][robot_id].publish(True)
            else:
                # 到達してないことをpublish
                self._pubs_is_arrived[color][robot_id].publish(False)
        else:
            rospy.logdebug("Velocity Control")
            command = self._make_command_velocity(color, robot_id, control_target.goal_velocity)

        # キック・ドリブルパラメータのセット
        command.kick_power = control_target.kick_power
        command.chip_enable = control_target.chip_enable
        command.dribble_power = control_target.dribble_power

        return command


    def _path_tracking(self, color, robot_id, path):
        ARRIVED_THRESH = 0.5 # meters

        arrived = False
        command = RobotCommand()
        command.robot_id = robot_id

        path_index = self._path_index[color][robot_id]
        robot_pose = self._robot_info[color][robot_id].pose

        if len(path) == 0 :
            # パスがセットされてない場合
            # 何もせず初期値のcommandを返す
            pass
        elif path_index == len(path):
            # ゴールまで到達した場合
            # 最後のposeに移動し続ける
            pose = path[-1]
            command = self._move_to_pose(color, robot_id, pose)

            # ゴールに到着したか判定
            if distance_2_poses(pose, robot_pose) < self._ARRIVED_THRESH:
                arrived = True
        else:
            # 経路追従　
            pose = path[path_index]
            command = self._move_to_pose(color, robot_id, pose)

            # poseに近づいたか判定する

            if distance_2_poses(pose, robot_pose) < self._APPROACH_THRESH:
                # 近づいたら次の経由位置へ向かう
                self._path_index[color][robot_id] += 1

        return command, arrived


    def _make_command_velocity(self, color, robot_id, goal_velocity):
        command = RobotCommand()
        command.robot_id = robot_id

        robot_info = self._robot_info[color][robot_id]
        current_control_velocity = self._control_velocity[color][robot_id]

        # 制御速度の生成
        robot_velocity = robot_info.velocity
        new_control_velocity = self._velocity_control(
                goal_velocity, current_control_velocity)

        # 速度方向をロボット座標系に変換
        theta = robot_info.pose.theta
        command.vel_surge = math.cos(theta)*new_control_velocity.x + math.sin(theta)*new_control_velocity.y
        command.vel_sway = -math.sin(theta)*new_control_velocity.x + math.cos(theta)*new_control_velocity.y
        command.vel_angular = new_control_velocity.theta

        # 制御速度の保存
        self._control_velocity[color][robot_id] = new_control_velocity

        return command


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
        pose_error = Pose2D()
        target_velocity = Pose2D()

        pose_error.x = goal_pose.x - robot_pose.x
        pose_error.y = goal_pose.y - robot_pose.y
        pose_error.theta = angle_normalize(goal_pose.theta - robot_pose.theta)

        # 目標動作速度を求める
        target_velocity = self._pid_controller[color][robot_id].update(pose_error, self._disable_integrater_input_x, self._disable_integrater_input_y, self._disable_integrater_input_theta,)
        new_control_velocity = self._velocity_control(target_velocity, current_control_velocity)

        # アンチワインドアップ処理
        # ref:  https://hamachannel.hatenablog.com/entry/2019/01/06/135004
        if (self.is_windup(target_velocity.x, new_control_velocity.x)):
            self._disable_integrater_input_x = True
        else:
            self._disable_integrater_input_x = False
        if (self.is_windup(target_velocity.y, new_control_velocity.y)):
            self._disable_integrater_input_y = True
        else:
            self._disable_integrater_input_y = False
        if (self.is_windup(target_velocity.theta, new_control_velocity.theta)):
            self._disable_integrater_input_theta = True
        else:
            self._disable_integrater_input_theta = False

        return new_control_velocity

    def is_windup(self, raw_vel, saturated_vel):
        if raw_vel >= 0:
            if raw_vel > saturated_vel:
                return True
        else:
            if raw_vel < saturated_vel:
                return True
        return False

    def _velocity_control(self, target_velocity, current_control_velocity):
        # 現在速度と目標速度の差分から、field座標系での制御速度を生成する

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
                self._MAX_ANGLE_ACCELERATION)

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

    
    def _acceleration_limit(self, target_velocity, current_velocity, limit_value):
        # 加速度制限をかけて目標速度を返す
        diff_velocity = target_velocity - current_velocity

        if math.fabs(diff_velocity) > limit_value:
            target_velocity = current_velocity + math.copysign(limit_value, diff_velocity)

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
                # 制御が有効な場合のみ動作司令を生成する
                if self._control_target[color][robot_info.robot_id].control_enable:
                    # 動作司令を生成
                    command = self._make_command(color, robot_info.robot_id)
                    robot_commands.commands.append(command)

                    # 動作司令の速度をpublish
                    self._pubs_command_velocity[color][robot_info.robot_id].publish(
                            self._control_velocity[color][robot_info.robot_id])
                else:
                    # 保存していた制御速度をリセットする
                    self._reset_control_velocity(color, robot_info.robot_id)

            # 動作司令が無ければpublishしない
            if len(robot_commands.commands) > 0:
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


