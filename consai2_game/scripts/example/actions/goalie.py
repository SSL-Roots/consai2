# coding: UTF-8

# defense.pyでは、ボールを蹴らないactionを定義する

import copy
import rospy
import math
import sys,os

from consai2_msgs.msg import BallInfo, RobotInfo
from consai2_msgs.msg import ControlTarget
from geometry_msgs.msg import Pose2D
import tool
import offense

sys.path.append(os.pardir)
from field import Field

# 直線の傾きと切片を算出
# 2点の座標から算出する
def _get_line_parameters(pose1, pose2):

    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y

    # 0になるとエラーになるのでその対策
    if x1 - x2 == 0:
        x1 += 1e-12
    
    # 傾きの算出
    slope = (y2 - y1) / (x2 - x1)
    # 切片の算出
    intercept = y2 - slope * x2
    
    return slope, intercept


# ゴーリーの位置を生成
def interpose(ball_info, robot_info, control_target):

    # ボールが動いていると判断するしきい値[m/s]
    MOVE_BALL_VELOCITY_THRESHOLD = 0.5
    # ロボットがボールを持っていると判断するしきい値[m]
    DIST_ROBOT_TO_BALL_THRESHOLD = 0.2
    # ゴールライン上ではなく一定距離[m]前を守るための変数
    MARGIN_DIST_X = 0.1

    # ゴールの位置(自陣のゴールのx座標は絶対負になる)
    OUR_GOAL_POSE = Field.goal_pose('our', 'center')
    OUR_GOAL_UPPER = Field.goal_pose('our', 'upper')
    OUR_GOAL_LOWER = Field.goal_pose('our', 'lower')

    # ボールの位置
    ball_pose = ball_info.pose

    # 敵のロボットの情報
    robot_info_their = robot_info['their']
    # 敵ロボットとボールの距離
    dist = []
    for their_info in robot_info_their:

        if their_info.disappeared is False:
            dist.append(tool.distance_2_poses(their_info.pose, ball_pose))
        else:
            dist.append(100)

    # 一番近い敵ロボットのID
    min_dist_id = dist.index(min(dist))

    # 一番近い敵ロボットの座標
    robot_pose = robot_info_their[min_dist_id].pose

    # ボールの速度
    ball_velocity_x = ball_info.velocity.x
    ball_velocity_y = ball_info.velocity.y
    ball_velocity = math.hypot(ball_velocity_x, ball_velocity_y)

    # ボールの進む角度
    angle_ball = math.atan2(ball_velocity_y, ball_velocity_x)
    # ボールの進む変化量を計算（方向を考慮した単位量）
    var_ball_velocity_x = math.cos(angle_ball) 
    var_ball_velocity_y = math.sin(angle_ball) 

    # ボールの次の予測位置を取得
    ball_pose_next = Pose2D(
            ball_pose.x + var_ball_velocity_x, ball_pose.y + var_ball_velocity_y, 0) 

    # 敵ロボットとボールの距離が近い場合は敵の向いている直線を使う
    if dist[min_dist_id] < DIST_ROBOT_TO_BALL_THRESHOLD and robot_pose.x < 0:
        slope = math.tan(robot_pose.theta)
        intercept = robot_pose.y - slope * robot_pose.x 

    # ボールの速度がある場合かつ近づいてくる場合はボールの向かう直線を使う
    elif MOVE_BALL_VELOCITY_THRESHOLD < ball_velocity and ball_velocity_x < 0:
        slope, intercept = _get_line_parameters(ball_pose, ball_pose_next)

    # その他はゴール中心とボールを結ぶ直線を使う
    else:
        slope, intercept = _get_line_parameters(ball_pose, OUR_GOAL_POSE)
    
    # ゴーリの新しい座標
    goalie_pose_x = OUR_GOAL_POSE.x + MARGIN_DIST_X
    goalie_pose_y = slope*goalie_pose_x + intercept

    # ゴールから飛び出さないようる上下限を設ける
    if OUR_GOAL_UPPER.y < goalie_pose_y:
        goalie_pose_y = OUR_GOAL_UPPER.y
    elif goalie_pose_y < OUR_GOAL_LOWER.y:
        goalie_pose_y = OUR_GOAL_LOWER.y

    # ゴーリの新しい座標
    new_goalie_pose = Pose2D(goalie_pose_x, goalie_pose_y, 0)

    # ---------------------------------------------------------
    remake_path = False
    # pathが設定されてなければpathを新規作成
    if control_target.path is None or len(control_target.path) == 0:
        remake_path = True
    # 現在のpathゴール姿勢と、新しいpathゴール姿勢を比較し、path再生成の必要を判断する
    if remake_path is False:
        current_goalie_pose = control_target.path[-1]

        if not tool.is_close(current_goalie_pose, new_goalie_pose, Pose2D(0.1, 0.1, math.radians(10))):
            remake_path = True

    # remake_path is Trueならpathを再生成する
    # pathを再生成すると衝突回避用に作られた経路もリセットされる
    if remake_path:
        control_target.path = []
        control_target.path.append(new_goalie_pose)

    return control_target


def demo_shoot(ball_info, robot_info, control_target, my_pose = None, inplay_shoot = False):

    # ゴールライン上ではなく一定距離[m]前を守るための変数
    MARGIN_DIST_X = 0.1

    # ゴールの位置(自陣のゴールのx座標は絶対負になる)
    OUR_GOAL_POSE = Field.goal_pose('our', 'center')
    OUR_GOAL_UPPER = Field.goal_pose('our', 'upper')
    OUR_GOAL_LOWER = Field.goal_pose('our', 'lower')

    ball_pose = ball_info.pose
    ball_velocity = ball_info.velocity
    ball_velocity_angle = tool.get_angle_from_center(ball_velocity)
    ball_vel_pose = Pose2D(
        ball_pose.x + 10 * math.cos(ball_velocity_angle),
        ball_pose.y + 10 * math.sin(ball_velocity_angle),
        0.0
    )

    new_goal_pose = Pose2D()
    # ボールが動いてたら、ディフェンスラインと、ボール速度の交点に移動
    defense_pose_upper = copy.deepcopy(OUR_GOAL_UPPER)
    defense_pose_upper.x += MARGIN_DIST_X
    defense_pose_lower = copy.deepcopy(OUR_GOAL_LOWER)
    defense_pose_lower.x += MARGIN_DIST_X

    intersection_pose = tool.get_intersection(
        ball_pose, ball_vel_pose, defense_pose_upper, defense_pose_lower)
    if intersection_pose \
        and tool.get_size_from_center(ball_info.velocity) > 0.5 \
        and math.fabs(intersection_pose.y) <= defense_pose_upper.y \
        and intersection_pose.x < defense_pose_lower.x + 0.1:
        new_goal_pose = intersection_pose
        new_goal_pose.x -= 0.09  # ロボットの中心からドリブラの位置までの距離を下げる:w
        new_goal_pose.theta = 0.0
        control_target.kick_power = 0.5
        control_target.dribble_power = 0.5
    else:
        # ボールが止まっていたら

        if inplay_shoot:
            return offense.inplay_shoot(my_pose, ball_info, control_target)

        new_goal_pose.x = OUR_GOAL_POSE.x + MARGIN_DIST_X
        new_goal_pose.y = 0.0
        new_goal_pose.theta = 0.0
        # to_ball_angle = tool.get_angle(new_goal_pose, ball_pose)
        # new_goal_pose.theta = to_ball_angle
        control_target.kick_power = 0.0
        control_target.dribble_power = 0.0

    control_target.path = []
    control_target.path.append(new_goal_pose)

    return control_target