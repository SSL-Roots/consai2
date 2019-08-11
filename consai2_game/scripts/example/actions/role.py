# coding: UTF-8

# role.pyでは、フィールド情報から各ロボットのmy_loleを定義する

# Roleの計算をするクラス
class RoleDecision(object):
    def __init__(self, robot_node, max_id, goalie_id):
        self._MAX_ID = max_id
        self._GOALIE_ID = goalie_id
        self._robot_node = robot_node
        self._rolestocker = RoleStocker(max_id)
        self._robot_disappeared = [False] * (max_id + 1)

        self._event_closest_to_the_ball = False
        self._event_ball_side = False
        self._event_robot_has_disappeared = False

    def set_disappeared(self, our_disappeared):
        if self._robot_disappeared != our_disappeared:
            self._event_robot_has_disappeared = True
        else:
            self._event_robot_has_disappeared = False
        self._robot_disappeared = our_disappeared

    def event_observer(self):
        if self._event_closest_to_the_ball or \
                self._event_ball_side or \
                self._event_robot_has_disappeared :
            self.update_role()

    def update_role(self):
        role_num = 1
        for robot_id in range(self._MAX_ID + 1):
            # ゴーリーを割り当てる
            if robot_id == self._GOALIE_ID:
                # 0はGoalie
                self._rolestocker.set_my_role(robot_id, 0)
            else:
                if self._robot_disappeared[robot_id] == False:
                    self._rolestocker.set_my_role(robot_id, role_num)
                    role_num += 1


# Roleを保持するクラス。RobotNodeはこいつを参照する
class RoleStocker(object):
    def __init__(self, max_id):
        self._my_role = [1] * (max_id + 1)

    def set_my_role(self, id, role_num):
        self._my_role[id] = role_num


def role_decision(robot_node):

    pass
