# coding: UTF-8
# role.pyでは、フィールド情報から各ロボットのmy_loleを定義する

from actions import tool

ROLE_ID = {
    "ROLE_GOALIE"        : 0,
    "ROLE_ATTACKER"      : 1,
    "ROLE_CENTER_BACK_1" : 2,
    "ROLE_CENTER_BACK_2" : 3,
    "ROLE_SUB_ATTACKER"  : 4,
    "ROLE_ZONE_1"        : 5,
    "ROLE_ZONE_2"        : 6,
    "ROLE_ZONE_3"        : 7,
    "ROLE_ZONE_4"        : 8,
    "ROLE_MAN_MARK_1"    : 9,
    "ROLE_MAN_MARK_2"    : 10,
    "ROLE_NONE"          : 99,
}

# TODO: 名前が正しくない
FIELD_PLAYER_NUM = 10

# Roleの計算をするクラス
class RoleDecision(object):
    def __init__(self, max_id, goalie_id):
        self._ROLE_MAX = len(ROLE_ID)
        self._FAR_DISTANCE = 1e+10

        self._MAX_ID = max_id
        self._GOALIE_ID = goalie_id

        self._robot_disappeared = [False] * (max_id + 1)
        self._role_is_exist = [False] * (self._ROLE_MAX + 1)
        self._dist_to_ball = [self._FAR_DISTANCE] * (max_id + 1)
        self._event_closest_to_the_ball = False
        self._event_ball_side = False
        self._event_robot_has_disappeared = False
        self._attacker_id = ROLE_ID["ROLE_ATTACKER"]
        self._attacker_id_pre = ROLE_ID["ROLE_ATTACKER"]
        self._attacker_had_disappeared = False

        self._rolestocker = RoleStocker(max_id, self._ROLE_MAX)

    def set_disappeared(self, our_disappeared):
        # 変化があった
        if self._robot_disappeared != our_disappeared:
            self._event_robot_has_disappeared = True
            for robot_id in range(self._MAX_ID + 1):
                robot_role = self._rolestocker.get_my_role(robot_id)
                if our_disappeared[robot_id] == True and \
                        robot_role != ROLE_ID["ROLE_NONE"]:
                    self._rolestocker.set_my_role(robot_id, ROLE_ID["ROLE_NONE"])
        else:
            self._event_robot_has_disappeared = False

        self._robot_disappeared = our_disappeared

    def check_ball_dist(self, our_pose, ball_info):
        # ロボットとボールの距離を計算して、attackerを決める
        MARGIN = 0.5 # meter
        change_attacker_id = False
        attacker_id_pre = self._attacker_id
        dist_min = self._FAR_DISTANCE

        for robot_id in range(self._MAX_ID + 1):
            if self._robot_disappeared[robot_id] == False and \
                    robot_id != self._GOALIE_ID:
                # ロボットとボールの距離を計算
                # もともとlast_detection_poseを使用していたが、不要と思われるのでposeにした
                dist = tool.distance_2_poses(our_pose[robot_id], 
                        ball_info.pose)
                # 距離の更新
                self._dist_to_ball[robot_id] = dist

                # アタッカーのボールロボット間距離を取得
                # マージンを設けて、アタッカーの切り替わりの発振を防ぐ
                closest_dist = self._dist_to_ball[self._attacker_id] - MARGIN

                # 前周期でアタッカーが死んでいた場合
                # 単純にボールに一番近いロボットをアタッカーに任命
                if self._attacker_had_disappeared:
                    if dist < dist_min:
                        dist_min = dist
                        self._attacker_id = robot_id
                        change_attacker_id = True
                # アタッカーが生きている場合
                elif dist < closest_dist:
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

        # アタッカー生死の判定更新
        self._attacker_had_disappeared = self._robot_disappeared[self._attacker_id]
    
    
    def event_observer(self):
        if self._event_closest_to_the_ball or \
                self._event_ball_side or \
                self._event_robot_has_disappeared :
            self.update_role()

    def update_role(self):
        # Goalie, Attaker 以外
        defense_start_num = ROLE_ID["ROLE_CENTER_BACK_1"]

        # Goalie
        self._rolestocker.set_my_role(self._GOALIE_ID, ROLE_ID["ROLE_GOALIE"])
        # Attaker
        if self._event_closest_to_the_ball:
            self._rolestocker.set_my_role(self._attacker_id_pre, ROLE_ID["ROLE_NONE"])
            self._rolestocker.set_my_role(self._attacker_id, ROLE_ID["ROLE_ATTACKER"])

        # 残りは優先度順に決定
        for role_id in range(defense_start_num, self._ROLE_MAX):
            if self._rolestocker.get_role_exist(role_id) == False:
                role_set_robot_id = -1
                role_before_set = ROLE_ID["ROLE_NONE"]
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
    def __init__(self, max_id, max_role):
        self._my_role = [ROLE_ID["ROLE_NONE"]] * (max_id + 1)
        self._role_is_exist = [False] * (max_role + 1)
        self._center_back_num = len([i for i in self._role_is_exist[ROLE_ID["ROLE_CENTER_BACK_1"]:
                (ROLE_ID["ROLE_CENTER_BACK_2"] + 1)] if i is True])
        self._zone_num = len([i for i in self._role_is_exist[ROLE_ID["ROLE_ZONE_1"]:
                (ROLE_ID["ROLE_ZONE_4"] + 1)] if i is True])
        self._defense_num = self._center_back_num + self._zone_num

    def set_my_role(self, robot_id, role_num):
        if self._my_role[robot_id] != ROLE_ID["ROLE_NONE"]:
            self._role_is_exist[self._my_role[robot_id]] = False 
        self._my_role[robot_id] = role_num
        if role_num != ROLE_ID["ROLE_NONE"]:
            self._role_is_exist[role_num] = True
        self._center_back_num = len([i for i in self._role_is_exist[ROLE_ID["ROLE_CENTER_BACK_1"]:
                (ROLE_ID["ROLE_CENTER_BACK_2"] + 1)] if i is True])
        self._zone_num = len([i for i in self._role_is_exist[ROLE_ID["ROLE_ZONE_1"]:
                (ROLE_ID["ROLE_ZONE_4"] + 1)] if i is True])
        self._defense_num = self._center_back_num + self._zone_num

    def get_my_role(self, robot_id):
        return self._my_role[robot_id]

    def get_role_exist(self, role_id):
        return self._role_is_exist[role_id]
