# coding: UTF-8

from actions import center_back, zone, man_mark
import role

def assign(my_role, ball_info, control_target, my_pose, defense_num, robot_info, zone_enable=False):
     # ゴール前ディフェンス
    if role.ROLE_ID['ROLE_CENTER_BACK_1'] <= my_role <= role.ROLE_ID['ROLE_CENTER_BACK_2']:
        return center_back.center_back(my_pose, ball_info, control_target, my_role, defense_num)
    elif my_role == role.ROLE_ID['ROLE_SUB_ATTACKER']:
        control_target.path = []
        control_target.path.append(my_pose)
        return control_target
    # ゾーンディフェンス
    elif role.ROLE_ID['ROLE_ZONE_1'] <= my_role <= role.ROLE_ID['ROLE_ZONE_4']:
        return zone.defense_zone(my_pose, ball_info, control_target, my_role, defense_num, robot_info['their'], zone_enable)
    elif role.ROLE_ID['ROLE_MAN_MARK_1'] <= my_role <= role.ROLE_ID['ROLE_MAN_MARK_2']:
        control_target.path = []
        control_target.path.append(my_pose)
        return control_target
    # 例外だった場合はその場にいる
    else:
        control_target.path = []
        control_target.path.append(my_pose)
        return control_target
