# coding: UTF-8

from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import math
import pk_tool
import rospy

class PkAttacker(object):
    _STATE_DISAPPEARED = 0
    _STATE_STANDBY = 1
    _STATE_APPROACH = 2
    _STATE_ROTATE = 3
    _STATE_AIM = 4

    def __init__(self, rostime_now):
        # 最大速度と最大加速度
        self._MAX_VELOCITY = Pose2D(0.5, 0.5, 2.0* math.pi)
        self._MAX_ACCELERATION = Pose2D(0.5/60.0, 0.5/60.0, 2.0*math.pi/60.0)

        # ボールに近づいた時、近づけたかを判定するしきい値。小さいほどきびしい
        self._APPROACH_DIST = 0.12  # meters
        # ボールに近づいた時、ボールを見ているか判定するしきい値。小さいほどきびしい
        self._APPROACH_ANGLE = 8.0  # degrees
        # ボールまわりで旋回する時、ターゲット（ゴール）を見ているか判定するしきい値
        # 小さいほどきびしい
        self._LOOK_TARGET_ANGLE = 5.0  # degrees

        # フットスイッチ長押し時に、キックフラグをリセットする時間
        # 5 をセットすると、5秒に１回キックフラグをFalseにする
        self._KICK_FLAG_RESET_TIME = 5  # seconds

        # 旋回時、シュート時のドリブルパワー
        self._DRIBBLE_POWER = 1.0  # 0.0 ~ 1.0
        self._KICK_POWER = 0.5  # 0.0 ~ 1.0
        # kazasuで操作するときの角速度
        self._AIM_OMEGA = 0.8  # rad/sec
        # kazasuで操作するときの、前進速度。
        self._AIM_VEL = 0.05  # meter/sec

        self._kick_timestamp = rostime_now
        self._current_state = self._STATE_DISAPPEARED
        self._has_kicked = False

    def get_control_target(self, my_robot_info, ball_info,
        field_length, field_width,
        kazasu_left, kazasu_right, foot_switch_has_pressed, rostime_now):
        control_target = ControlTarget()

        if my_robot_info.disappeared:
            # ロボットが消えたら強制的に目標速度0の司令を返す
            self._current_state = self._STATE_DISAPPEARED
            control_target.goal_velocity = Pose2D(0, 0, 0)
            return control_target
        elif self._current_state == self._STATE_DISAPPEARED:
            # ロボットが存在してたらスタンバイ状態に戻す
            self._current_state = self._STATE_STANDBY

        my_pose = my_robot_info.pose
        ball_pose = ball_info.pose
        target_pose = Pose2D(-field_length * 0.5, 0, 0)  # 相手ゴールの中心をシュート目標にする

        # ロボットがボールから離れたら、
        if pk_tool.distance_2_poses(my_pose, ball_pose) > 0.5:
            # キックした場合はSTANBYに戻す
            if self._has_kicked:
                self._current_state = self._STATE_STANDBY

            elif self._current_state != self._STATE_STANDBY:
                # それ以外はAPPROACHにもどす
                self._current_state = self._STATE_APPROACH

        if self._current_state == self._STATE_STANDBY:
            # kazasuが両方反応したらデモを開始する
            if kazasu_left and kazasu_right:
                self._current_state = self._STATE_APPROACH
                self._has_kicked = False

        elif self._current_state == self._STATE_APPROACH:
            # 今いる位置からボールにまっすぐ近づく
            control_target, self._current_state = self._approach(my_pose, ball_pose)

        elif self._current_state == self._STATE_ROTATE:
            # ボールに回り込み、targetを見る
            control_target, self._current_state = self._rotate(
                my_pose, ball_pose, target_pose)

        elif self._current_state == self._STATE_AIM:
            # ちょっとずつ前進しながら、kazasu信号で回転し、foot_swでシュート
            control_target = self._aim_and_shoot(ball_pose, target_pose,
                kazasu_left, kazasu_right, foot_switch_has_pressed, rostime_now)

        # 最大速度と加速度に制限を設ける
        control_target.max_velocity.append(self._MAX_VELOCITY)
        control_target.max_acceleration.append(self._MAX_ACCELERATION)
        return control_target

    def _approach(self, my_pose, ball_pose):
        # 今いる位置からボールにまっすぐ近づく
        control_target = ControlTarget()
        next_state = self._STATE_APPROACH

        angle_ball_to_robot = pk_tool.get_angle(ball_pose, my_pose)
        trans_BtoR = pk_tool.Trans(ball_pose, angle_ball_to_robot)

        control_target_pose = trans_BtoR.inverted_transform(Pose2D(-0.1, 0, 0))
        control_target_pose.theta = trans_BtoR.inverted_transform_angle(math.radians(180))
        control_target.path.append(control_target_pose)

        # ロボットがボールに近づき、ボールを見つめていたら状態遷移する
        dist_to_ball = trans_BtoR.transform(my_pose).x
        look_angle = math.fabs(trans_BtoR.transform_angle(my_pose.theta))
        if dist_to_ball < self._APPROACH_DIST\
            and look_angle > math.radians(180 - self._APPROACH_ANGLE):
            next_state = self._STATE_ROTATE

        return control_target, next_state

    def _rotate(self, my_pose, ball_pose, target_pose):
        # ボールに回り込み、targetを見る
        control_target = ControlTarget()
        next_state = self._STATE_ROTATE

        angle_ball_to_robot = pk_tool.get_angle(ball_pose, my_pose)
        trans_BtoR = pk_tool.Trans(ball_pose, angle_ball_to_robot)
        angle_ball_to_target = pk_tool.get_angle(ball_pose, target_pose)
        trans_BtoT = pk_tool.Trans(ball_pose, angle_ball_to_target)

        tr_robot_angle_BtoT = trans_BtoT.transform_angle(my_pose.theta)

        length = trans_BtoR.transform(my_pose).x
        add_angle = math.copysign(math.radians(45), tr_robot_angle_BtoT) * -1.0
        tr_goal_pose_BtoR = Pose2D(length*math.cos(add_angle), length*math.sin(add_angle), 0)

        control_target_pose = trans_BtoR.inverted_transform(tr_goal_pose_BtoR)
        control_target_pose.theta = pk_tool.get_angle(control_target_pose, ball_pose)
        control_target.dribble_power = self._DRIBBLE_POWER
        control_target.path.append(control_target_pose)

        # ロボットがゴールを見つめたら状態遷移する
        if math.fabs(tr_robot_angle_BtoT) < math.radians(self._LOOK_TARGET_ANGLE):
            next_state = self._STATE_AIM

        return control_target, next_state

    def _aim_and_shoot(self, ball_pose, target_pose,
        kazasu_left, kazasu_right, foot_sw_on, rostime_now):
        # ちょっとずつ前進しながら、kazasu信号で回転する
        control_target = ControlTarget()

        angle_ball_to_target = pk_tool.get_angle(ball_pose, target_pose)

        omega = 0.0
        if kazasu_left:
            omega += self._AIM_OMEGA
        if kazasu_right:
            omega -= self._AIM_OMEGA
        control_target.goal_velocity = Pose2D(
            self._AIM_VEL * math.cos(angle_ball_to_target),
            self._AIM_VEL * math.sin(angle_ball_to_target),
            omega)
        control_target.dribble_power = self._DRIBBLE_POWER

        if foot_sw_on:
            # フットスイッチが押されたらキック
            control_target.kick_power = self._KICK_POWER
            self._has_kicked = True

            # フットスイッチが長押しされてる時、数秒に１回、キックフラグをリセットする
            if rostime_now.to_sec() - self._kick_timestamp.to_sec()  > self._KICK_FLAG_RESET_TIME:
                control_target.kick_power = 0.0
                self._kick_timestamp = rostime_now
        else:
            self._kick_timestamp = rostime_now

        return control_target

