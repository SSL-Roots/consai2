# coding: UTF-8

# role.pyでは、フィールド情報から各ロボットのmy_loleを定義する

# Roleの計算をするクラス
class RoleDecision(object):
    def __init__(self, robot_node, max_id, goalie_id):
        self._ROLE_MAX = 8

        self._MAX_ID = max_id
        self._GOALIE_ID = goalie_id
        self._robot_node = robot_node

        self._robot_disappeared = [False] * (max_id + 1)
        self._role_is_exist = [False] * (self._ROLE_MAX + 1)
        self._event_closest_to_the_ball = False
        self._event_ball_side = False
        self._event_robot_has_disappeared = False
        self._attacker_id = 1

        self._rolestocker = RoleStocker(max_id)

    def set_disappeared(self, our_disappeared):
        # 変化があった
        if self._robot_disappeared != our_disappeared:
            self._event_robot_has_disappeared = True
            for robot_id in range(self._MAX_ID + 1):
                robot_role = self._rolestocker.get_my_role(robot_id)
                if our_disappeared[robot_id] == True and \
                        robot_role != 99:
                    self._role_is_exist[robot_role] = False
                    self._rolestocker.set_my_role(robot_id, 99)
        else:
            self._event_robot_has_disappeared = False

        self._robot_disappeared = our_disappeared

    

    def event_observer(self):
        if self._event_closest_to_the_ball or \
                self._event_ball_side or \
                self._event_robot_has_disappeared :
            self.update_role()

    def update_role(self):
        # Goalie, Attaker 以外
        defence_start_num = 2

        # Goalie
        self._rolestocker.set_my_role(self._GOALIE_ID, 0)
        # Attaker
        self._rolestocker.set_my_role(self._attacker_id, 1)
        # Defence
        for role_id in range(defence_start_num, self._ROLE_MAX):
            if self._role_is_exist[role_id] == False:
                role_set_robot_id = -1
                role_before_set = 99
                max_role_id = role_id
                for robot_id in range(self._MAX_ID + 1):
                    # 生きているロボット
                    if self._robot_disappeared[robot_id] == False:
                        robot_role = self._rolestocker.get_my_role(robot_id)
                        if robot_role > max_role_id:
                            max_role_id = robot_role
                            role_set_robot_id = robot_id
                            role_before_set = robot_role
                if role_set_robot_id != -1:
                    self._rolestocker.set_my_role(role_set_robot_id, role_id)
                    self._role_is_exist[role_id] = True
                    if role_before_set != 99:
                        self._role_is_exist[role_before_set] = False


# Roleを保持するクラス。RobotNodeはこいつを参照する
class RoleStocker(object):
    def __init__(self, max_id):
        self._my_role = [99] * (max_id + 1)

    def set_my_role(self, id, role_num):
        self._my_role[id] = role_num

    def get_my_role(self, id):
        return self._my_role[id]


def role_decision(robot_node):

    pass
