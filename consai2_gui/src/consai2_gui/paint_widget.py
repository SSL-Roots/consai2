# coding: UTF-8

import rospy
import math

from python_qt_binding.QtCore import Qt, QPointF, QRectF
from python_qt_binding.QtGui import QPainter, QPen ,QColor
from python_qt_binding.QtGui import QMouseEvent
from python_qt_binding.QtWidgets import QWidget

from consai2_msgs.msg import VisionGeometry


class PaintWidget(QWidget):
    def __init__(self, parent=None):
        super(PaintWidget, self).__init__(parent)

        self._WHITE_LINE_THICKNESS = 2 # 白線の太さ
        self._ZOOM_RATE = 0.1 # 拡大縮小率
        self._SCALE_LIMIT = 0.2 # 縮小率の限界値

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

        # フィールド形状
        # raw_vision_geometryの値で更新される
        self._field_length = 9.0
        self._field_width = 6.0
        self._field_goal_width = 1.0
        self._field_goal_depth = 0.2
        self._field_boundary_width = 0.3
        self._field_lines = []
        self._field_arcs = []

        # Subscribers
        self._sub_geometry = rospy.Subscriber(
                'vision_receiver/raw_vision_geometry', VisionGeometry, 
                self._callback_geometry, queue_size=1)


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


    def mousePressEvent(self, event):
        # マウスのドラッグ操作で描画領域を移動する
        # 右クリックで移動と拡大縮小をリセットする
        if event.buttons() == Qt.LeftButton:
            self._click_point = event.localPos()

        elif event.buttons() == Qt.RightButton:
            self._reset_painter_status()

        self.update()


    def mouseMoveEvent(self, event):
        # マウスのドラッグ操作で描画領域を移動する
        self._current_mouse_pos = event.localPos()

        if event.buttons() == Qt.LeftButton:
            self._mouse_trans = (
                    self._current_mouse_pos - self._click_point) / self._scale.x()

        self.update()


    def mouseReleaseEvent(self, event):
        # マウスのドラッグ操作で描画領域を移動する
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

    
    def _reset_painter_status(self):
        # 描画領域の移動と拡大縮小を初期化する
        self._trans = QPointF(0.0, 0.0)
        self._mouse_trans = QPointF(0.0, 0.0)
        self._scale = QPointF(1.0, 1.0)


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


