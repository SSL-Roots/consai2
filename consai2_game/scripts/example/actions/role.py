# coding: UTF-8
# role.pyでは、フィールド情報から各ロボットのmy_loleを定義する

from actions import tool


# Roleの計算をするクラス
class RoleDecision(object):
    def __init__(self, max_id, goalie_id):
        self._ROLE_MAX = 8
        self._FAR_DISTANCE = 1e+10
        self._ROLE_IS_NONE = 99

        self._MAX_ID = max_id
        self._GOALIE_ID = goalie_id

        self._robot_disappeared = [False] * (max_id + 1)
        self._role_is_exist = [False] * (self._ROLE_MAX + 1)
        self._dist_to_ball = [self._FAR_DISTANCE] * (max_id + 1)
        self._event_closest_to_the_ball = False
        self._event_ball_side = False
        self._event_robot_has_disappeared = False
        self._attacker_id = 1
        self._attacker_id_pre = 1

        self._rolestocker = RoleStocker(max_id, self._ROLE_MAX, self._ROLE_IS_NONE)

    def set_disappeared(self, our_disappeared):
        # 変化があった
        if self._robot_disappeared != our_disappeared:
            self._event_robot_has_disappeared = True
            for robot_id in range(self._MAX_ID + 1):
                robot_role = self._rolestocker.get_my_role(robot_id)
                if our_disappeared[robot_id] == True and \
                        robot_role != self._ROLE_IS_NONE:
                    # self._role_is_exist[robot_role] = False
                    self._rolestocker.set_my_role(robot_id, self._ROLE_IS_NONE)
        else:
            self._event_robot_has_disappeared = False

        self._robot_disappeared = our_disappeared

    def check_ball_dist(self, our_pose, ball_info):
        # ロボットとボールの距離を計算して、attackerを決める
        MARGIN = 0.5 # meter
        change_attacker_id = False
        attacker_id_pre = self._attacker_id

        for robot_id in range(self._MAX_ID + 1):
            if self._robot_disappeared[robot_id] == False and \
                    robot_id != self._GOALIE_ID:
                # ロボットとボールの距離を計算
                # ボールが消える可能性を考慮して、last_detection_poseを使う
                dist = tool.distance_2_poses(our_pose[robot_id], 
                        ball_info.last_detection_pose)
                # 距離の更新
                self._dist_to_ball[robot_id] = dist

                # アタッカーのボールロボット間距離を取得
                # マージンを設けて、アタッカーの切り替わりの発振を防ぐ
                closest_dist = self._dist_to_ball[self._attacker_id] - MARGIN

                if dist < closest_dist:
                    self._attacker_id = robot_id
                    change_attacker_id = True
            else:
                if self._attacker_id == robot_id:
                    self._dist_to_ball[robot_id] = self._FAR_DISTANCE
        if change_attacker_id:
            self._attacker_id_pre = attacker_id_pre
            self._event_closest_to_the_ball = True
        else:
            self._event_closest_to_the_ball = False

    
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
        if self._event_closest_to_the_ball:
            self._rolestocker.set_my_role(self._attacker_id_pre, self._ROLE_IS_NONE)
            self._rolestocker.set_my_role(self._attacker_id, 1)

        # Defence
        for role_id in range(defence_start_num, self._ROLE_MAX):
            if self._rolestocker.get_role_exist(role_id) == False:
                role_set_robot_id = -1
                role_before_set = self._ROLE_IS_NONE
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


# Roleを保持するクラス。RobotNodeはこいつを参照する
class RoleStocker(object):
    def __init__(self, max_robot, max_role, role_is_none):
        self._role_is_none = role_is_none
        self._my_role = [self._role_is_none] * (max_robot + 1)
        self._role_is_exist = [False] * (max_role + 1)

    def set_my_role(self, robot_id, role_num):
        if self._my_role[robot_id] != self._role_is_none:
            self._role_is_exist[self._my_role[robot_id]] = False 
        self._my_role[robot_id] = role_num
        if role_num != self._role_is_none:
            self._role_is_exist[role_num] = True

    def get_my_role(self, robot_id):
        return self._my_role[robot_id]

    def get_role_exist(self, role_id):
        return self._role_is_exist[role_id]
