#!/usr/bin/env python2
# coding: UTF-8

import rospy
import math
import serial

from consai2_msgs.msg import RobotCommands

class RealSender(object):
    def __init__(self):
        self._DEVICE  = rospy.get_param('~device', '/dev/ttyUSB0')
        self._BAUDRATE   = rospy.get_param('~baudrate', 57600)

        self._MAX_VEL_NORM = 4.0 # m/s
        self._MAX_VEL_ANGULAR = 2.0*math.pi

        self._serial = serial.Serial(self._DEVICE, self._BAUDRATE)

        self._sub_commands = rospy.Subscriber(
                'consai2_control/robot_commands',
                RobotCommands,
                self._send,
                queue_size = 10)


    def _send(self, msg):
        # 実機ロボットに動作司令を送信する
        # Roots Protocol
        # 0: 1111 1111 |HEADER_1 0xFF
        # 1: 1100 0011 |HEADER_2 0xC3
        # 2: 0000 xxxx |x:ID
        # 3: aaaa aaaa |a:vel_norm
        # 4: bbbb bbbb |b:vel_theta
        # 5: cccc cccc |c:omega(0~254), 127->0, 254->2PI [rad/sec]
        # 6: d01e f110 |d:dribble_flag, e:kick_flag, f:chip_enable
        # 7: gggg hhhh |g:dribble_power, h:kick_power
        # 8: **** **** |XOR([2] ~ [7])
        # 9: **** **** |XOR([8],0xFF)

        for command in msg.commands:
        # for command in self._robot_commands.commands:
            packet = bytearray()

            # ヘッダー
            packet.append(0xFF)
            packet.append(0xC3)
        
            # ロボットID
            packet.append(command.robot_id)
        
            # 走行速度ノルム
            # 0 ~ max_vel -> 0 ~ 254
            vel_norm = math.sqrt(
                    math.pow(command.vel_surge, 2) + 
                    math.pow(command.vel_sway,2))

            if vel_norm > self._MAX_VEL_NORM:
                vel_norm = self._MAX_VEL_NORM
            elif vel_norm < 0:
                vel_norm = 0
            # 0 ~ max_vel -> 0 ~ 254
            packet.append(int(254 * (vel_norm/self._MAX_VEL_NORM)))

            # 走行速度方向
            # -pi ~ pi -> 0 ~ 179
            vel_theta = math.atan2(command.vel_surge, -command.vel_sway)

            # -pi ~ pi -> 0 ~ 2pi
            if vel_theta < 0:
                vel_theta += 2.0 * math.pi
            vel_theta = math.degrees(vel_theta)
            vel_theta += 0 # -0.0を除去する
            # 0 ~ 2pi -> 0 ~ 179
            packet.append(int(vel_theta/2.0))


            # 走行角速度
            # -2pi ~ pi -> 0 ~ 127 ~ 254
            # -2pi -> 0
            # 0 -> 127
            # 2pi -> 254
            vel_angular = command.vel_angular
            if math.fabs(vel_angular) > self._MAX_VEL_ANGULAR:
                vel_angular = math.copysign(self._MAX_VEL_ANGULAR, vel_angular)

            # -2pi ~ pi -> 0 ~ 127 ~ 254
            packet.append(int(127*(vel_angular/self._MAX_VEL_ANGULAR) + 127))

            # キックとドリブル
            # 6: d01e f110 |d:dribble_flag, e:kick_flag, f:chip_enable
            # 7: gggg hhhh |g:dribble_power, h:kick_power
            command_packet = 0
            power_packet = 0

            # ドリブル
            # 0 ~ 1.0 -> 0 ~ 15
            if command.dribble_power > 0:
                command_packet += 0x80
                dribble_power = command.dribble_power
                if dribble_power > 1.0:
                    dribble_power = 1.0
                elif dribble_power < 0:
                    dribble_power = 0
                dribble_power = int(15 * dribble_power)

                power_packet += dribble_power << 4

            # キック
            # 0 ~ 1.0 -> 0 ~ 15
            if command.kick_power > 0:
                command_packet += 0x10
                kick_power = command.kick_power
                if kick_power > 1.0:
                    kick_power = 1.0
                elif kick_power < 0:
                    kick_power = 0
                kick_power = int(15 * kick_power)

                power_packet += kick_power

            command_packet += 0x26

            packet.append(command_packet)
            packet.append(power_packet)

            # チェックサム
            # 8: **** **** |XOR([2] ~ [7])
            # 9: **** **** |XOR([8],0xFF)
            check_sum = 0

            for p in packet[2:]:
                check_sum ^= p

            packet.append(check_sum)
            packet.append(check_sum ^ 0xFF)

            self._serial.write(packet)
        

    def close_serial(self):
        # 通信ポートを閉じる
        self._serial.close()


def main():
    rospy.init_node('real_sender')
    sender = RealSender()

    rospy.on_shutdown(sender.close_serial)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass

