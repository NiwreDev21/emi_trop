# joystick_widget.py
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
import math

class JoystickWidget(QWidget):
    angleChanged = pyqtSignal(float)
    mousePressed = pyqtSignal()
    mouseReleased = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(150, 150)
        self.setMaximumSize(200, 200)
        
        self.center = QPoint(0, 0)
        self.thumb_position = QPoint(0, 0)
        self.radius = 50
        self.thumb_radius = 15
        self.is_pressed = False
        self.yaw = 0.0
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        self.center = QPoint(self.width() // 2, self.height() // 2)
        self.radius = min(self.width(), self.height()) // 2 - 10
        
        painter.setPen(QPen(QColor("#444"), 2))
        painter.setBrush(QBrush(QColor("#333")))
        painter.drawEllipse(self.center, self.radius, self.radius)
        
        painter.setPen(QPen(QColor("#666"), 1, Qt.DashLine))
        painter.drawLine(self.center.x() - self.radius, self.center.y(), 
                        self.center.x() + self.radius, self.center.y())
        painter.drawLine(self.center.x(), self.center.y() - self.radius,
                        self.center.x(), self.center.y() + self.radius)
        
        thumb_color = QColor("#00CCCC")
        if self.is_pressed:
            thumb_color = QColor("#44BBBB")
        
        painter.setPen(QPen(QColor("#00AAAA"), 2))
        painter.setBrush(QBrush(thumb_color))
        painter.drawEllipse(self.thumb_position, self.thumb_radius, self.thumb_radius)
        
        painter.setPen(QPen(QColor("#eee"), 1))
        painter.setBrush(QBrush(QColor("#eee")))
        painter.drawEllipse(self.center, 3, 3)
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_pressed = True
            self.mousePressed.emit()
            self.update_thumb_position(event.pos())
            
    def mouseMoveEvent(self, event):
        if self.is_pressed:
            self.update_thumb_position(event.pos())
            
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_pressed = False
            self.mouseReleased.emit()
            self.update()
            
    def update_thumb_position(self, pos):
        vector = pos - self.center
        distance = math.sqrt(vector.x()**2 + vector.y()**2)
        
        if distance > self.radius:
            vector = vector * (self.radius / distance)
            distance = self.radius
            
        self.thumb_position = self.center + vector
        
        # Calcular ángulo CORREGIDO para que coincida con la dirección de la cámara
        # Invertir el eje Y para que coincida con el sistema de coordenadas de Gazebo
        self.yaw = math.degrees(math.atan2(vector.x(), -vector.y()))
        
        self.angleChanged.emit(self.yaw)
        self.update()
        
    def get_angle(self):
        return self.yaw
        
    def reset(self):
        self.thumb_position = self.center
        self.yaw = 0.0
        self.update()
        self.angleChanged.emit(0.0)