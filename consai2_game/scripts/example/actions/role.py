# coding: UTF-8

# role.pyでは、フィールド情報から各ロボットのmy_loleを定義する


class Event(object):
    def __init__(self):
        self._closest_to_the_ball = False
        self._ball_side = False
        self._number_of_robots = False


# Roleの計算をするクラス
class RoleDecision(object):
    def __init__(self, robot_node, max_id, goalie_id):
        self._robot_node = robot_node
        self._rolestocker = RoleStocker(max_id, goalie_id)


# Roleを保持するクラス。RobotNodeはこいつを参照する
class RoleStocker(object):
    def __init__(self, max_id, goalie_id):
        self._MAX_ID = max_id
        self._GOALIE_ID = goalie_id

        self._my_role = []

        role_num = 1
        for robot_id in range(self._MAX_ID + 1):
            # ゴーリーを割り当てる
            if robot_id == self._GOALIE_ID:
                self._my_role.append(0)
            else:
                self._my_role.append(role_num)
                role_num += 1


def role_decision(robot_node):

    pass
