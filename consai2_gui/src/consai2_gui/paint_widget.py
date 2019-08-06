# coding: UTF-8

import rospy
import math
import copy

from python_qt_binding.QtCore import Qt, QPointF, QRectF
from python_qt_binding.QtGui import QPainter, QPen ,QColor
from python_qt_binding.QtGui import QMouseEvent
from python_qt_binding.QtWidgets import QWidget

from consai2_msgs.msg import VisionGeometry, BallInfo, RobotInfo
from consai2_msgs.msg import Replacements, ReplaceBall, ReplaceRobot
from consai2_msgs.msg import ControlTarget


class PaintWidget(QWidget):
    def __init__(self, parent=None):
        super(PaintWidget, self).__init__(parent)

        self._WHITE_LINE_THICKNESS = 2 # 白線の太さ
        self._ZOOM_RATE = 0.1 # 拡大縮小率
        self._SCALE_LIMIT = 0.2 # 縮小率の限界値
        self._ALPHA_DETECTED = 255 # 検出できたロボット・ボールの透明度
        self._ALPHA_NOT_DETECTED  = 127 # 検出できなかったロボット・ボールの透明度
        self._COLOR_BALL = QColor(Qt.red)
        self._COLOR_ROBOT  = {'blue':QColor(Qt.cyan), 'yellow':QColor(Qt.yellow)}
        self._ID_POS = (0.15, 0.15) # IDを描画するロボット中心からの位置

        # Replace
        self._REPLACE_CLICK_POS_THRESHOLD = 0.1
        self._REPLACE_CLICK_VEL_ANGLE_THRESHOLD = self._REPLACE_CLICK_POS_THRESHOLD + 0.1
        self._REPLACE_BALL_VELOCITY_GAIN = 3.0
        self._REPLACE_MAX_BALL_VELOCITY = 8.0

        self._BALL_RADIUS = rospy.get_param('consai2_description/ball_radius', 0.0215)
        self._ROBOT_RADIUS = rospy.get_param('consai2_description/robot_radius', 0.09)
        self._MAX_ID = rospy.get_param('consai2_description/max_id', 15)
        self._SIDE = rospy.get_param('consai2_description/our_side', 'left')

        # チームサイドの反転
        self._invert_side = False
        if self._SIDE != 'left':
            self._invert_side = True

        # GUIパラメータ
        self._trans      = QPointF(0.0, 0.0) # x, y方向の移動
        self._mouse_trans = QPointF(0.0, 0.0) # マウス操作による移動
        self._scale      = QPointF(1.0, 1.0) # 拡大, 縮小
        self._do_rotate_view = False # 90度回転判定
        self._view_height = self.height() # 描画サイズ（縦）
        self._view_width = self.width() # 描画サイズ（横）
        self._scale_field_to_view = 1.0 # フィールドから描画領域に縮小するスケール
        self._click_point = QPointF(0.0, 0.0) # マウスでクリックした描画座標
        self._current_mouse_pos = QPointF(0.0, 0.0) # マウスカーソル位置
        self._replace_func = None
        self._replace_id = 0
        self._replace_is_yellow = False
        self._do_replacement = False
        self._replacement_target = {'ball_pos':False, 'ball_vel':False,
                'robot_pos':False, 'robot_angle':False}

        # フィールド形状
        # raw_vision_geometryの値で更新される
        self._field_length = 9.0
        self._field_width = 6.0
        self._field_goal_width = 1.0
        self._field_goal_depth = 0.2
        self._field_boundary_width = 0.3
        self._field_lines = []
        self._field_arcs = []

        # ジョイスティック情報
        self._joy_target = ControlTarget()

        # ロボット・ボール情報
        self._ball_info = BallInfo()
        self._robot_info = {'blue':[],'yellow':[]}

        # Publisher
        self._pub_replace = rospy.Publisher('sim_sender/replacements', 
                Replacements, queue_size=1)

        # Subscribers
        self._sub_geometry = rospy.Subscriber(
                'vision_receiver/raw_vision_geometry', VisionGeometry, 
                self._callback_geometry, queue_size=1)

        self._sub_ball_info = rospy.Subscriber(
                'vision_wrapper/ball_info', BallInfo,
                self._callback_ball_info, queue_size=1)

        self._sub_joy_target = rospy.Subscriber(
                'consai2_examples/joy_target', ControlTarget, 
                self._callback_joy_target, queue_size=1)

        self._subs_robot_info = {'blue':[], 'yellow':[]}

        for robot_id in range(self._MAX_ID +1):
            # 末尾に16進数の文字列をつける
            topic_id = hex(robot_id)[2:]
            topic_name = 'vision_wrapper/robot_info_blue_' + topic_id
            self._subs_robot_info['blue'].append(
                    rospy.Subscriber(topic_name, RobotInfo,
                        self._callback_blue_info, callback_args=robot_id))

            topic_name = 'vision_wrapper/robot_info_yellow_' + topic_id
            self._subs_robot_info['yellow'].append(
                    rospy.Subscriber(topic_name, RobotInfo,
                        self._callback_yellow_info, callback_args=robot_id))

            self._robot_info['blue'].append(RobotInfo())
            self._robot_info['yellow'].append(RobotInfo())

        # Configs
        # This function enables mouse tracking without pressing mouse button
        self.setMouseTracking(True)

    def _callback_geometry(self, msg):
        # フィールド形状を更新
        if msg.field_length:
            self._field_length = msg.field_length

        if msg.field_width:
            self._field_width = msg.field_width

        if msg.goal_width:
            self._field_goal_width = msg.goal_width

        if msg.goal_depth:
            self._field_goal_depth = msg.goal_depth

        if msg.boundary_width:
            self._field_boundary_width = msg.boundary_width

        if msg.field_lines:
            self._field_lines = []
            for line in msg.field_lines:
                self._field_lines.append(
                        {"name":line.name, "p1_x":line.p1_x, "p1_y":line.p1_y,
                            "p2_x":line.p2_x, "p2_y":line.p2_y, "thickness":line.thickness})
        if msg.field_arcs:
            self._field_arcs = []
            for arc in msg.field_arcs:
                self._field_arcs.append(
                    {"name":arc.name, "center_x":arc.center_x, "center_y":arc.center_y,
                        "radius":arc.radius, "a1":arc.a1, "a2":arc.a2, "thickness":arc.thickness})

        self._resize_draw_world()


    def _callback_ball_info(self, msg):
        self._ball_info = msg

    def _callback_blue_info(self, msg, robot_id):
        self._robot_info['blue'][robot_id] = msg

    def _callback_yellow_info(self, msg, robot_id):
        self._robot_info['yellow'][robot_id] = msg

    def _callback_joy_target(self, msg):
        self._joy_target = msg


    def mousePressEvent(self, event):
        # マウスのドラッグ操作で描画領域を移動する
        # 右クリックで移動と拡大縮小をリセットする
        if event.buttons() == Qt.LeftButton:
            self._click_point = event.localPos()

            self._do_replacement = self._is_replacement_click(self._click_point)

        elif event.buttons() == Qt.RightButton:
            self._reset_painter_status()

        self.update()


    def mouseMoveEvent(self, event):
        # マウスのドラッグ操作で描画領域を移動する
        # Replacementを行うときは描画領域を移動しない
        self._current_mouse_pos = event.localPos()

        if self._do_replacement:
            pass
        elif event.buttons() == Qt.LeftButton:
            self._mouse_trans = (
                    self._current_mouse_pos - self._click_point) / self._scale.x()

        self.update()


    def mouseReleaseEvent(self, event):
        # マウスのドラッグ操作で描画領域を移動する
        # Replacementを行うときは描画領域を移動しない
        if self._do_replacement:
            self._do_replacement = False
            self._replace_func(event.localPos())
        else:
            self._trans += self._mouse_trans
            self._mouse_trans = QPointF(0.0, 0.0)
        
        self.update()


    def wheelEvent(self, event):
        # マウスのホイール操作で描画領域を拡大縮小する
        s = self._scale.x()
        if event.angleDelta().y() > 0:
            self._scale.setX(s + self._ZOOM_RATE)
            self._scale.setY(s + self._ZOOM_RATE)
        else:
            if s > self._SCALE_LIMIT:
                self._scale.setX(s - self._ZOOM_RATE)
                self._scale.setY(s - self._ZOOM_RATE)

        self.update()


    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx,cy)

        painter.scale(self._scale.x(), self._scale.y())
        painter.translate(self._trans + self._mouse_trans)

        if self._do_rotate_view is True:
            painter.rotate(-90)

        # これ以降に書きたいものを重ねる
        self._draw_field(painter)
        self._draw_ball(painter)
        self._draw_ball_velocity(painter)

        # JoyStick関連
        if len(self._joy_target.path) > 0:
            self._draw_joy_target(painter)

        self._draw_robots(painter)

        # grSim Replacement関連
        if self._replacement_target['ball_pos'] or self._replacement_target['robot_pos']:
            self._draw_pos_replacement(painter)
            self._draw_cursor_coordinate(painter)

        elif self._replacement_target['ball_vel']:
            self._draw_vel_replacement(painter)

        elif self._replacement_target['robot_angle']:
            self._draw_angle_replacement(painter)

        else:
            self._draw_cursor_coordinate(painter)



    def resizeEvent(self, event):
        self._resize_draw_world()


    def _resize_draw_world(self):
        # Widgetのサイズに合わせて、描くフィールドのサイズを変える
        # 描画の回転判断もしてくれるすぐれもの

        # Widgetの縦横比を算出
        widget_height = float(self.height())
        widget_width = float(self.width())
        widget_w_per_h = widget_width / widget_height

        # Fieldの縦横比を算出
        field_width = self._field_length + self._field_boundary_width * 2.0
        field_height = self._field_width + self._field_boundary_width * 2.0
        field_w_per_h = field_width / field_height
        field_h_per_w = 1.0 / field_w_per_h

        if widget_w_per_h >= field_w_per_h:
            # Widgetが横長のとき
            self._view_height = widget_height
            self._view_width = widget_height * field_w_per_h
            self._do_rotate_view = False

        elif widget_w_per_h <= field_h_per_w:
            # Widgetが縦長のとき
            self._view_height = widget_width
            self._view_width = widget_width * field_w_per_h
            self._do_rotate_view = True

        else:
            # 描画回転にヒステリシスをもたせる
            if self._do_rotate_view is True:
                self._view_height = widget_height * field_h_per_w
                self._view_width = widget_height
            else:
                self._view_height = widget_width * field_h_per_w
                self._view_width = widget_width

        self._scale_field_to_view = self._view_width / field_width

    
    def _convert_to_view(self, x, y):
        # フィールド座標系を描画座標系に変換する
        view_x = x * self._scale_field_to_view
        view_y = -y * self._scale_field_to_view
        point = QPointF(view_x, view_y)
        return point

    
    def _convert_to_field(self, x, y):
        # 描画座標系をフィールド座標系に変換する
        x /= self._scale.x()
        y /= self._scale.y()

        x -= (self._trans.x() + self._mouse_trans.x())
        y -= (self._trans.y() + self._mouse_trans.y())

        x -= self.width() * 0.5 / self._scale.x()
        y -= self.height() * 0.5 / self._scale.y()

        if self._do_rotate_view:
            x, y = -y, x

        field_x = x / self._scale_field_to_view
        field_y = -y / self._scale_field_to_view
        point = QPointF(field_x, field_y)

        return point

    
    def _reset_painter_status(self):
        # 描画領域の移動と拡大縮小を初期化する
        self._trans = QPointF(0.0, 0.0)
        self._mouse_trans = QPointF(0.0, 0.0)
        self._scale = QPointF(1.0, 1.0)


    def _is_replacement_click(self, mouse_pos):
        # クリックした描画位置にオブジェクトがあればReplacementと判定する
        # ボールとロボットが近い場合、ボールのReplacementを優先する
        field_point = self._convert_to_field(mouse_pos.x(), mouse_pos.y())

        is_clicked = True

        result = self._is_ball_clicked(field_point)
        if result == 'pos':
            self._replacement_target['ball_pos'] = True
            self._replace_func = self._replace_ball_pos
        elif result == 'vel_angle':
            self._replacement_target['ball_vel'] = True
            self._replace_func = self._replace_ball_vel
        else:
            result, robot_id, is_yellow = self._is_robot_clicked(field_point)
            self._replace_id = robot_id
            self._replace_is_yellow = is_yellow

            if result == 'pos':
                self._replacement_target['robot_pos'] = True
                self._replace_func = self._replace_robot_pos
            elif result == 'vel_angle':
                self._replacement_target['robot_angle'] = True
                self._replace_func = self._replace_robot_angle
            else:
                is_clicked = False

        return is_clicked


    def _is_clicked(self, field_point1, field_point2):
        # フィールド上のオブジェクトをクリックしたかどうか判定する
        diff_point = field_point1 - field_point2
        diff_norm = math.hypot(diff_point.x(), diff_point.y())
        if diff_norm < self._REPLACE_CLICK_POS_THRESHOLD:
            return 'pos'
        elif diff_norm < self._REPLACE_CLICK_VEL_ANGLE_THRESHOLD:
            return 'vel_angle'

        return False


    def _is_ball_clicked(self, field_point):
        # ボールをクリックしたか判定する
        # ボールが消えていれば判定しない

        if self._ball_info.disappeared:
            return False

        pos_x = self._ball_info.pose.x
        pos_y = self._ball_info.pose.y
        ball_pos = QPointF(pos_x, pos_y)

        return self._is_clicked(field_point, ball_pos)

    def _is_robot_clicked(self, field_point):
        # ロボットをクリックしたか判定する
        # 消えたロボットは対照外

        is_clicked = False
        replace_id = 0
        is_yellow = False

        for robot in self._robot_info['blue']:
            if robot.disappeared:
                continue
            robot_point = QPointF(robot.pose.x, robot.pose.y)
            is_clicked = self._is_clicked(field_point, robot_point)

            if is_clicked:
                is_yellow = False
                return is_clicked, robot.robot_id, is_yellow

        for robot in self._robot_info['yellow']:
            if robot.disappeared:
                continue
            robot_point = QPointF(robot.pose.x, robot.pose.y)
            is_clicked = self._is_clicked(field_point, robot_point)

            if is_clicked:
                is_yellow = True
                return is_clicked, robot.robot_id, is_yellow

        return is_clicked, replace_id, is_yellow


    def _replace_ball_pos(self, mouse_pos):
        # ボール位置のReplacement
        field_point = self._convert_to_field(mouse_pos.x(), mouse_pos.y())

        ball = ReplaceBall()
        ball.x = field_point.x()
        ball.y = field_point.y()
        ball.is_enabled = True

        if self._invert_side:
            ball.x *= -1.0
            ball.y *= -1.0

        replacements = Replacements()
        replacements.ball = ball
        self._pub_replace.publish(replacements)
        self._replacement_target['ball_pos'] = False


    def _replace_ball_vel(self, mouse_pos):
        # ボール速度のReplacement
        ball_point = QPointF(self._ball_info.pose.x, self._ball_info.pose.y)
        field_point = self._convert_to_field(mouse_pos.x(), mouse_pos.y())
        velocity = self._replacement_velocity(ball_point, field_point)

        ball = ReplaceBall()
        ball.x = ball_point.x()
        ball.y = ball_point.y()
        ball.vx = velocity.x()
        ball.vy = velocity.y()
        ball.is_enabled = True

        if self._invert_side:
            ball.x *= -1.0
            ball.y *= -1.0
            ball.vx *= -1.0
            ball.vy *= -1.0

        replacements = Replacements()
        replacements.ball = ball
        self._pub_replace.publish(replacements)
        self._replacement_target['ball_vel'] = False


    def _replacement_velocity(self, from_point, to_point):
        # Replacementのボール速度を計算する
        diff_point = to_point - from_point
        diff_norm = math.hypot(diff_point.x(), diff_point.y())

        velocity_norm = diff_norm * self._REPLACE_BALL_VELOCITY_GAIN
        if velocity_norm > self._REPLACE_MAX_BALL_VELOCITY:
            velocity_norm = self._REPLACE_MAX_BALL_VELOCITY

        angle = math.atan2(diff_point.y(), diff_point.x())
        velocity = QPointF(
                velocity_norm * math.cos(angle),
                velocity_norm * math.sin(angle))

        return velocity


    def _replace_robot_pos(self, mouse_pos):
        # ロボット位置のReplacement
        field_point = self._convert_to_field(mouse_pos.x(), mouse_pos.y())

        # ロボット角度をradiansからdegreesに変換する
        direction = 0
        if self._replace_is_yellow:
            direction = math.degrees(self._robot_info['yellow'][self._replace_id].pose.theta)
        else:
            direction = math.degrees(self._robot_info['blue'][self._replace_id].pose.theta)

        robot = ReplaceRobot()
        robot.x = field_point.x()
        robot.y = field_point.y()
        robot.dir = direction
        robot.id = self._replace_id
        robot.yellowteam = self._replace_is_yellow
        robot.turnon = True

        if self._invert_side:
            robot.x *= -1.0
            robot.y *= -1.0
            robot.dir += 180

        replacements = Replacements()
        replacements.robots.append(robot)
        self._pub_replace.publish(replacements)
        self._replacement_target['robot_pos'] = False


    def _replace_robot_angle(self, mouse_pos):
        # ロボット角度のReplacement
        field_point = self._convert_to_field(mouse_pos.x(), mouse_pos.y())

        # ロボット角度をradiansからdegreesに変換する
        robot_point = QPointF()
        if self._replace_is_yellow:
            robot_point = QPointF(
                    self._robot_info['yellow'][self._replace_id].pose.x,
                    self._robot_info['yellow'][self._replace_id].pose.y)
        else:
            robot_point = QPointF(
                    self._robot_info['blue'][self._replace_id].pose.x,
                    self._robot_info['blue'][self._replace_id].pose.y)

        robot = ReplaceRobot()
        robot.x = robot_point.x()
        robot.y = robot_point.y()
        robot.dir = math.degrees(self._to_angle(robot_point, field_point))
        robot.id = self._replace_id
        robot.yellowteam = self._replace_is_yellow
        robot.turnon = True

        if self._invert_side:
            robot.x *= -1.0
            robot.y *= -1.0
            robot.dir += 180

        replacements = Replacements()
        replacements.robots.append(robot)
        self._pub_replace.publish(replacements)
        self._replacement_target['robot_angle'] = False


    def _draw_field(self, painter):
        # フィールドの緑と白線を描画する

        # グリーンカーペットを描画
        painter.setPen(Qt.black)
        painter.setBrush(Qt.green)

        rect = QRectF(-self._view_width*0.5, -self._view_height*0.5, 
                self._view_width, self._view_height)
        painter.drawRect(rect)

        # 白線を描画
        painter.setPen(QPen(Qt.white, self._WHITE_LINE_THICKNESS))

        for line in self._field_lines:
            p1 = self._convert_to_view(line["p1_x"], line["p1_y"])
            p2 = self._convert_to_view(line["p2_x"], line["p2_y"])
            painter.drawLine(p1, p2)

        for arc in self._field_arcs:
            top_left = self._convert_to_view(
                    arc["center_x"] - arc["radius"],
                    arc["center_y"] + arc["radius"] )
            size = arc["radius"] * 2.0 * self._scale_field_to_view

            # angle must be 1/16 degrees order
            start_angle = math.degrees(arc["a1"]) * 16
            end_angle = math.degrees(arc["a2"]) * 16
            span_angle = end_angle - start_angle
            painter.drawArc(top_left.x(), top_left.y(), size, size, start_angle, span_angle)

    
    def _draw_ball(self, painter):
        # ボールを描画する

        if self._ball_info.disappeared is False:
            point = self._convert_to_view(
                    self._ball_info.pose.x, self._ball_info.pose.y)
            size = self._BALL_RADIUS * self._scale_field_to_view

            ball_color = copy.deepcopy(self._COLOR_BALL)
            if self._ball_info.detected is False:
                # ボールを検出してないときは透明度を変える
                ball_color.setAlpha(self._ALPHA_NOT_DETECTED)
            painter.setPen(Qt.black)
            painter.setBrush(ball_color)
            painter.drawEllipse(point, size, size)

    
    def _draw_ball_velocity(self, painter):
        # ボールの速度方向を描画する
        VELOCITY_THRESH = 1.0
        PAINT_DIST = 10.0 # meters

        ball_pos = self._ball_info.pose
        ball_vel = self._ball_info.velocity

        # 速度が小さければ描画しない
        if math.hypot(ball_vel.x, ball_vel.y) < VELOCITY_THRESH:
            return

        direction = math.atan2(ball_vel.y, ball_vel.x)

        vel_pos_x = PAINT_DIST * math.cos(direction) + ball_pos.x
        vel_pos_y = PAINT_DIST * math.sin(direction) + ball_pos.y

        point1 = self._convert_to_view(ball_pos.x, ball_pos.y)
        point2 = self._convert_to_view(vel_pos_x, vel_pos_y)

        painter.setPen(QPen(QColor(102, 0, 255), 2))
        painter.drawLine(point1, point2)



    def _draw_robots(self, painter):
        # 全てのロボットを描画する

        for blue in self._robot_info['blue']:
            self._draw_robot(painter, blue, 'blue')
            
        for yellow in self._robot_info['yellow']:
            self._draw_robot(painter, yellow, 'yellow')


    def _draw_robot(self, painter, robot, color):
        # ロボット1台を描画する

        if robot.disappeared is False:
            point = self._convert_to_view(robot.pose.x, robot.pose.y)
            size = self._ROBOT_RADIUS * self._scale_field_to_view

            robot_color = copy.deepcopy(self._COLOR_ROBOT[color])
            if robot.detected is False:
                # ロボットを検出してないときは透明度を変える
                robot_color.setAlpha(self._ALPHA_NOT_DETECTED)
            painter.setPen(Qt.black)
            painter.setBrush(robot_color)
            painter.drawEllipse(point, size, size)

            # ロボット角度
            line_x = self._ROBOT_RADIUS * math.cos(robot.pose.theta)
            line_y = self._ROBOT_RADIUS * math.sin(robot.pose.theta)
            line_point = point + self._convert_to_view(line_x, line_y)
            painter.drawLine(point, line_point)

            # ロボットID
            text_point = point + self._convert_to_view(self._ID_POS[0], self._ID_POS[1])
            painter.drawText(text_point, str(robot.robot_id))


    def _draw_cursor_coordinate(self, painter):
        # マウスカーソルのフィールド座標を描画する
        current_pos = self._convert_to_field(
                self._current_mouse_pos.x(), self._current_mouse_pos.y())
        current_point = self._convert_to_view(current_pos.x(), current_pos.y())

        text = "(" + str(round(current_pos.x(),2)) + ", " + str(round(current_pos.y(),2)) + ")"

        painter.setPen(Qt.black)
        painter.drawText(current_point, text)


    def _draw_pos_replacement(self, painter):
        # 位置のReplacementを描画する
        start_pos = self._convert_to_field(
                self._click_point.x(), self._click_point.y())
        current_pos = self._convert_to_field(
                self._current_mouse_pos.x(), self._current_mouse_pos.y())

        start_point = self._convert_to_view(start_pos.x(), start_pos.y())
        current_point = self._convert_to_view(current_pos.x(), current_pos.y())

        painter.setPen(QPen(Qt.red, 2))
        painter.drawLine(start_point, current_point)


    def _draw_vel_replacement(self, painter):
        # ボール速度のReplacementを描画する

        current_pos = self._convert_to_field(
                self._current_mouse_pos.x(), self._current_mouse_pos.y())
        current_point = self._convert_to_view(current_pos.x(), current_pos.y())
        ball_pos = QPointF(
                self._ball_info.pose.x, self._ball_info.pose.y)
        ball_point = self._convert_to_view(ball_pos.x(), ball_pos.y())

        painter.setPen(QPen(Qt.blue, 2))
        painter.drawLine(ball_point, current_point)

        # 速度の数値を描画する
        velocity = self._replacement_velocity(ball_pos, current_pos)
        text = "[" + str(round(velocity.x(),2)) + ", " + str(round(velocity.y(),2)) + "]"
        painter.setPen(Qt.black)
        painter.drawText(current_point, text)


    def _draw_angle_replacement(self, painter):
        # ロボット角度のReplacementを描画する
        robot_pos = QPointF()
        if self._replace_is_yellow:
            robot_pos.setX(self._robot_info['yellow'][self._replace_id].pose.x)
            robot_pos.setY(self._robot_info['yellow'][self._replace_id].pose.y)
        else:
            robot_pos.setX(self._robot_info['blue'][self._replace_id].pose.x)
            robot_pos.setY(self._robot_info['blue'][self._replace_id].pose.y)
        robot_point = self._convert_to_view(robot_pos.x(), robot_pos.y())

        current_pos = self._convert_to_field(
                self._current_mouse_pos.x(), self._current_mouse_pos.y())
        current_point = self._convert_to_view(current_pos.x(), current_pos.y())

        painter.setPen(QPen(Qt.blue, 2))
        painter.drawLine(robot_point, current_point)

        # 角度の数値を描画する
        angle = math.degrees(self._to_angle(robot_pos, current_pos))
        text = "[" + str(round(angle,2)) + "]"
        painter.setPen(Qt.black)
        painter.drawText(current_point, text)

    
    def _to_angle(self, from_point, to_point):
        diff_point = to_point - from_point

        return math.atan2(diff_point.y(), diff_point.x())


    def _draw_joy_target(self, painter):
        # JoyStickのControlTargetを描画する

        # 経路の線を描画するための変数
        prev_point = None

        for i, pose in enumerate(self._joy_target.path):
            point = self._convert_to_view(pose.x, pose.y)
            size = self._ROBOT_RADIUS * self._scale_field_to_view

            joy_color = QColor(Qt.magenta)
            painter.setPen(Qt.black)
            painter.setBrush(joy_color)
            painter.drawEllipse(point, size, size)

            # 角度
            line_x = self._ROBOT_RADIUS * math.cos(pose.theta)
            line_y = self._ROBOT_RADIUS * math.sin(pose.theta)
            line_point = point + self._convert_to_view(line_x, line_y)
            painter.drawLine(point, line_point)

            # インデックス
            text_point = point + self._convert_to_view(self._ID_POS[0], self._ID_POS[1])
            painter.drawText(text_point, str(i))

            # 経路を描画
            if prev_point is None:
                prev_point = point
            else:
                painter.setPen(QPen(QColor(0,0,255, 127), 4))
                painter.drawLine(prev_point, point)
                prev_point = point


