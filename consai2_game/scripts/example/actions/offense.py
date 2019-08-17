# coding: UTF-8

# offense.pyでは、ボールを蹴るactionを定義する

import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool

sys.path.append(os.pardir)
from field import Field


def simple_kick(my_pose, ball_info, control_target, kick_power=0.5):
    # ボールへまっすぐ向かい、そのまま蹴る
    # シュート目標位置を設定しないシンプルなaction

    
    # 目標位置はボールの前方にし、目標角度は、自己位置からみたボール方向にする
    angle_robot_to_ball = tool.get_angle(my_pose, ball_info.pose)
    trans = tool.Trans(ball_info.pose, angle_robot_to_ball)
    tr_position= Pose2D(0.2, 0, 0) # ボールより少し前を目標位置にする
    position = trans.inverted_transform(tr_position)

    new_goal_pose = Pose2D()
    new_goal_pose = position
    new_goal_pose.theta = angle_robot_to_ball

    # ボールに近づいたらキックフラグをON
    if tool.distance_2_poses(my_pose, ball_info.pose) < 0.5:
        control_target.kick_power = kick_power
    else:
        control_target.kick_power = 0.0

    # ドリブルは常にオフ
    control_target.dribble_power = 0.0

    # ---------------------------------------------------------
    remake_path = False
    # pathが設定されてなければpathを新規作成
    if control_target.path is None or len(control_target.path) == 0:
        remake_path = True
    # 現在のpathゴール姿勢と、新しいpathゴール姿勢を比較し、path再生成の必要を判断する
    if remake_path is False:
        current_goal_pose = control_target.path[-1]

        if not tool.is_close(current_goal_pose, new_goal_pose, Pose2D(0.1, 0.1, math.radians(10))):
            remake_path = True
    # remake_path is Trueならpathを再生成する
    # pathを再生成すると衝突回避用に作られた経路もリセットされる
    if remake_path:
        control_target.path = []
        control_target.path.append(new_goal_pose)

    return control_target


def _inplay_shoot(my_pose, ball_info, control_target, target_pose,
        can_shoot_angle = 5):
    # インプレイ用のシュートアクション
    # デフォルトでゴールを狙う

    KICK_POWER = 1.0
    IS_TOUCH_DIST = 0.2 # meters
    IS_TOUCH_ANGLE = 160 # degrees
    IS_LOOK_TARGET_ANGLE = 45 # degrees
    CAN_SHOOT_ANGLE = can_shoot_angle # degrees
    SHOOT_TARGET = target_pose

    # ボールから見たロボットの座標系を生成
    angle_ball_to_robot = tool.get_angle(ball_info.pose, my_pose)
    trans_BtoR = tool.Trans(ball_info.pose, angle_ball_to_robot)
    tr_robot_pose_BtoR = trans_BtoR.transform(my_pose)
    tr_robot_angle_BtoR = trans_BtoR.transform_angle(my_pose.theta)

    # ボールから見たターゲットの座標系を生成
    angle_ball_to_target = tool.get_angle(ball_info.pose, SHOOT_TARGET)
    trans_BtoT = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_robot_angle_BtoT = trans_BtoT.transform_angle(my_pose.theta)

    can_look_target = False
    can_shoot = False
    # ドリブラーがボールにくっついたらcan_look_target is True
    # ロボットとボールの直線距離が近いか
    if tr_robot_pose_BtoR.x < IS_TOUCH_DIST \
            and math.fabs(tr_robot_angle_BtoR) > math.radians(IS_TOUCH_ANGLE): #ロボットがボールを見ているか
        can_look_target = True

    # ロボットがTargetを向いたらcan_shoot is True
    if math.fabs(tr_robot_angle_BtoT) < math.radians(IS_LOOK_TARGET_ANGLE):
        can_shoot = True

    new_goal_pose = Pose2D()
    if can_look_target is False:
        # ドリブラーがボールにつくまで移動する
        rospy.logdebug("inplay_shoot: approach")

        new_goal_pose = trans_BtoR.inverted_transform(Pose2D(-0.1, 0, 0))
        new_goal_pose.theta = trans_BtoR.inverted_transform_angle(math.radians(180))

        # ドリブルとキックをオフ
        control_target.kick_power = 0.0
        control_target.dribble_power = 0.0
    elif can_shoot is False:
        # 目標角度に値を加えてロボットを回転させる
        rospy.logdebug("inplay_shoot: rotate")

        # ドリブラーがボールにつくまで移動する
        tr_robot_pose_BtoR = trans_BtoR.transform(my_pose)
        length = tr_robot_pose_BtoR.x
        ADD_ANGLE = math.copysign(90, tr_robot_angle_BtoT) * -1.0
        tr_goal_pose_BtoR = Pose2D(length*math.cos(ADD_ANGLE), length*math.sin(ADD_ANGLE), 0)

        # ボールにくっつきながら回転動作を加える
        new_goal_pose = trans_BtoR.inverted_transform(tr_goal_pose_BtoR)
        new_goal_pose.theta = tool.get_angle(new_goal_pose, ball_info.pose)

        # ドリブルをオン、キックをオフ
        control_target.kick_power = 0.0
        control_target.dribble_power = 1.0
    else:

        new_goal_pose = trans_BtoT.inverted_transform(Pose2D(0.01, 0, 0))
        new_goal_pose.theta = angle_ball_to_target
        # ドリブルをオフ、キックをオン

        # 狙いが定まったらシュート
        if math.fabs(tr_robot_angle_BtoT) < math.radians(CAN_SHOOT_ANGLE):
            rospy.logdebug("inplay_shoot: shoot")

            control_target.kick_power = KICK_POWER
            control_target.dribble_power = 1.0
        else:
            rospy.logdebug("inplay_shoot: pre-shoot")
            control_target.kick_power = 0.0
            control_target.dribble_power = 1.0



    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)


    return control_target


def inplay_shoot(my_pose, ball_info, control_target):
    # インプレイ用のシュートアクション
    # デフォルトでゴールを狙う
    SHOOT_TARGET = Field.goal_pose('their', 'center')
    CAN_SHOOT_ANGLE = 10 # degrees

    return _inplay_shoot(my_pose, ball_info, control_target, SHOOT_TARGET, CAN_SHOOT_ANGLE)


def outside_shoot(my_pose, ball_info, control_target):
    # ゴールに入らないように外側にボールをけるアクション

    OUR_GOAL = Field.goal_pose('our', 'center')
    TARGET_LENGTH = 4.0
    CAN_SHOOT_ANGLE = 10 # degrees

    shoot_target = Pose2D()
    angle_goal_to_ball = tool.get_angle(OUR_GOAL, ball_info.pose)

    trans = tool.Trans(ball_info.pose, angle_goal_to_ball)
    shoot_target = trans.inverted_transform(Pose2D(TARGET_LENGTH, 0, 0))

    return _inplay_shoot(my_pose, ball_info, control_target, shoot_target, CAN_SHOOT_ANGLE)


def _setplay_shoot(my_pose, ball_info, control_target, kick_enable, target_pose, kick_power=0.8, receive_enable=False, receiver_role_exist=False, robot_info=None):
    # セットプレイ用のシュートアクション
    # kick_enable is Falseで、ボールの近くまで移動する
    # kick_enable is True で、シュートする

    KICK_POWER = kick_power
    SHOOT_TARGET = target_pose

    arrive_threshold = 0.2

    angle_ball_to_target = tool.get_angle(ball_info.pose, SHOOT_TARGET)
    trans = tool.Trans(ball_info.pose, angle_ball_to_target)
    tr_my_pose = trans.transform(my_pose)

    # ロボットがボールの裏側に回ったらcan_kick is True
    can_kick = False
    if tr_my_pose.x < 0.01 and math.fabs(tr_my_pose.y) < 0.05:
        can_kick = True

    # レシーバにパスする場合、蹴る位置近くにロボットが存在すれば receive_arrive is True
    # ただし、指定したroleが存在しなければ関係なし
    # can_kick と receive_arrive が両方Trueなら蹴る
    if receive_enable and receiver_role_exist:
        receive_arrive = False
        for robot_id in range(len(robot_info['our'])):
            if arrive_threshold > tool.distance_2_poses(target_pose, robot_info['our'][robot_id].pose):
                receive_arrive = True
        can_kick = can_kick and receive_arrive

    avoid_ball = True # ボールを回避する
    new_goal_pose = Pose2D()
    if can_kick and kick_enable:
        # ボールをける
        avoid_ball = False

        # ボールの前方に移動する
        new_position = trans.inverted_transform(Pose2D(0.02, 0, 0))
        new_goal_pose = new_position
        new_goal_pose.theta = angle_ball_to_target
        # ドリブルとキックをオン
        control_target.kick_power = KICK_POWER
    else:
        # ボールの裏に移動する
        new_position = trans.inverted_transform(Pose2D(-0.3, 0, 0))
        new_goal_pose = new_position
        new_goal_pose.theta = angle_ball_to_target
        # ドリブルとキックをオフ
        control_target.kick_power = 0.0
        control_target.dribble_power = 0.0
        
    # パスを追加
    control_target.path = []
    control_target.path.append(new_goal_pose)


    return control_target, avoid_ball

def setplay_shoot(my_pose, ball_info, control_target, kick_enable=False):
    # セットプレイ用のシュートアクション
    # kick_enable is Falseで、ボールの近くまで移動する
    # kick_enable is True で、シュートする

    SHOOT_TARGET = Field.goal_pose('their', 'center')

    return _setplay_shoot(my_pose, ball_info, control_target, kick_enable, SHOOT_TARGET)

def setplay_pass(my_pose, ball_info, control_target, target_pose, receive_enable=False, receiver_role_exist=None, robot_info=None, direct=False):

    kick_enable = True
    kick_power = 0.5

    # ダイレクトかつ、直接シュートが無理の無い位置だった場合は直シュート
    if direct and ball_info.pose.x < Field.penalty_pose('their','upper_front').x:
        return _setplay_shoot(my_pose, ball_info, control_target, kick_enable, Field.goal_pose('their','center'), kick_power)

    # receive_enable ならレシーバにパスする動きをする
    if receive_enable:
        return _setplay_shoot(my_pose, ball_info, control_target, kick_enable, target_pose,kick_power,
                receive_enable, receiver_role_exist, robot_info)
    # それ以外ならターゲットに蹴るだけ
    else:
        return _setplay_shoot(my_pose, ball_info, control_target, kick_enable, target_pose,kick_power)
