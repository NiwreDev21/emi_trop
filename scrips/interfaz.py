#!/usr/bin/env python3
import sys
import os
import rospy
import cv2
import subprocess
import math
from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QSizePolicy, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage, QIcon

import tf.transformations as tft
import numpy as np


class DroneGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador EMI - Vista Observador")
        self.setGeometry(100, 100, 1200, 800)
        self.setWindowIcon(QIcon("/home/niwre21/catkin_ws/src/emi_trop/scripts/dronicoono.png"))

        self.bridge = CvBridge()
        self.image_obs = None

        self.altitude = 0.0
        self.battery = 0.0
        self.speed = 0.0
        self.mode = "DESCONOCIDO"
        self.armed = False
        
        # Posición fija de la cámara según tu archivo .world
        self.camera_fixed_position = np.array([0.67, -0.98, 0.96])
        
        # Variables para la posición del dron
        self.drone_position = np.array([0.0, 0.0, 0.0])
        
        self.sub_obs = None
        self.sub_alt = None
        self.sub_batt = None
        self.sub_speed = None
        self.sub_state = None
        self.sub_local_pose = None

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.start_roscore()
        self.init_ui()
        self.init_ros()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_images)
        self.timer.start(50)
        
        # Timer para actualizar la orientación de la cámara
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_camera_orientation)
        self.camera_timer.start(100)  # Actualizar cada 100ms

    def start_roscore(self):
        def run_roscore():
            subprocess.Popen(
                ["roscore"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setpgrp
            )
        Thread(target=run_roscore, daemon=True).start()

    def init_ui(self):
        self.setStyleSheet("""
            QWidget {
                background: #1b1b1b; 
                color: #eee; 
                font-family: 'Segoe UI', Tahoma;
                font-size: 14px;
            }
            QLabel#cameraLabel {
                font-size: 16px; 
                color: #bbb;
                background-color: #2b2b2b;
                border: 2px solid #444;
                border-radius: 5px;
                padding: 10px;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #444;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
                background-color: #2b2b2b;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #00CCCC;
            }
            QFrame {
                background-color: #222;
                border-radius: 5px;
            }
        """)

        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # --- Vista del Observador ---
        self.label_obs = QLabel("SIN SEÑAL (Observer Camera)")
        self.label_obs.setObjectName("cameraLabel")
        self.label_obs.setAlignment(Qt.AlignCenter)
        self.label_obs.setFrameShape(QFrame.Box)
        self.label_obs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label_obs.setMinimumSize(800, 500)
        main_layout.addWidget(self.label_obs)

        # --- Botón de activación de cámara ---
        button_style = """
        QPushButton {
            background-color: #00CCCC;
            color: #000;
            border: 2px solid #00AAAA;
            border-radius: 5px;
            padding: 10px 15px;
            font-weight: bold;
            font-size: 14px;
        }
        QPushButton:hover {
            background-color: #66DDDD;
        }
        QPushButton:pressed {
            background-color: #44BBBB;
        }
        """
        
        self.btn_cam = QPushButton("Activar cámara")
        self.btn_cam.setStyleSheet(button_style)
        self.btn_cam.clicked.connect(self.toggle_subs)
        main_layout.addWidget(self.btn_cam)

        # Información de la cámara
        cam_group = QGroupBox("Cámara de Observador")
        cam_layout = QVBoxLayout()
        cam_layout.setSpacing(10)
        
        cam_info = QLabel("Posición fija: (0.67, -0.98, 0.96)\nSigue automáticamente al dron")
        cam_info.setStyleSheet("color: #aaa; padding: 5px;")
        cam_layout.addWidget(cam_info)
        
        cam_group.setLayout(cam_layout)
        main_layout.addWidget(cam_group)

        # Telemetría
        navbar = QFrame()
        navbar.setFixedHeight(50)
        telemetry_layout = QHBoxLayout()
        self.label_alt = QLabel("Altitud: -- m")
        self.label_batt = QLabel("Batería: -- %")
        self.label_speed = QLabel("Velocidad: -- m/s")
        self.label_mode = QLabel("Modo: -- | Armado: --")
        for lbl in [self.label_alt, self.label_batt, self.label_speed, self.label_mode]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-size: 14px; padding: 5px; background-color: #333; border-radius: 3px;")
            telemetry_layout.addWidget(lbl)
        navbar.setLayout(telemetry_layout)
        main_layout.addWidget(navbar)

    def init_ros(self):
        rospy.init_node('drone_gui_pyqt', anonymous=True, disable_signals=True)

    def toggle_subs(self):
        if not self.sub_obs:
            self.sub_obs = rospy.Subscriber("/observer_camera/observer_camera/image_raw", Image, self.cb_observer)
            self.sub_alt = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_pose)
            self.sub_batt = rospy.Subscriber("/mavros/battery", BatteryState, self.cb_battery)
            self.sub_speed = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.cb_velocity)
            self.sub_state = rospy.Subscriber("/mavros/state", State, self.cb_state)
            self.sub_local_pose = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)
            self.btn_cam.setText("Desactivar cámara")
        else:
            for sub in [self.sub_obs, self.sub_alt, self.sub_batt, 
                        self.sub_speed, self.sub_state, self.sub_local_pose]:
                if sub: sub.unregister()
            self.sub_obs = self.sub_alt = self.sub_batt = None
            self.sub_speed = self.sub_state = self.sub_local_pose = None
            self.btn_cam.setText("Activar cámara")
            self.label_obs.setText("SIN SEÑAL (Observer Camera)")
            self.label_alt.setText("Altitud: -- m")
            self.label_batt.setText("Batería: -- %")
            self.label_speed.setText("Velocidad: -- m/s")
            self.label_mode.setText("Modo: -- | Armado: --")

    def cb_observer(self, data):
        try: 
            self.image_obs = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e: 
            rospy.logwarn(f"Observer Error: {e}")

    def cb_pose(self, msg):
        self.altitude = msg.pose.position.z
        self.label_alt.setText(f"Altitud: {self.altitude:.2f} m")

    def cb_battery(self, msg):
        self.battery = msg.percentage * 100
        self.label_batt.setText(f"Batería: {self.battery:.0f} %")

    def cb_velocity(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        self.speed = (vx**2 + vy**2 + vz**2)**0.5
        self.label_speed.setText(f"Velocidad: {self.speed:.2f} m/s")

    def cb_state(self, msg):
        self.mode = msg.mode
        self.armed = msg.armed
        armed_text = "True" if self.armed else "False"
        self.label_mode.setText(f"Modo: {self.mode} | Armado: {armed_text}")

    def cb_local_pose(self, msg):
        # Obtener la posición del dron
        self.drone_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def cv_to_pixmap_responsive(self, cv_img, label):
        if cv_img is None: 
            return None
        try:
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            scaled = qt_img.scaled(label.width(), label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            return QPixmap.fromImage(scaled)
        except Exception as e:
            rospy.logwarn(f"Error converting image: {e}")
            return None

    def refresh_images(self):
        if self.image_obs is not None: 
            pixmap = self.cv_to_pixmap_responsive(self.image_obs, self.label_obs)
            if pixmap:
                self.label_obs.setPixmap(pixmap)

    def update_camera_orientation(self):
        """Actualiza solo la orientación de la cámara para que mire hacia el dron"""
        if hasattr(self, 'drone_position'):
            # Posición fija de la cámara (definida en tu .world)
            cam_x, cam_y, cam_z = self.camera_fixed_position
            
            # Calcular la dirección hacia el dron
            dx = self.drone_position[0] - cam_x
            dy = self.drone_position[1] - cam_y
            dz = self.drone_position[2] - cam_z
            
            # Calcular yaw y pitch para que la cámara mire al dron
            yaw_cam = math.atan2(dy, dx)
            pitch_cam = -math.atan2(dz, math.sqrt(dx**2 + dy**2))
            
            # Crear la pose de la cámara (posición fija, solo cambia orientación)
            pose = Pose()
            pose.position.x = cam_x
            pose.position.y = cam_y
            pose.position.z = cam_z
            
            # Convertir los ángulos a cuaternión
            q = tft.quaternion_from_euler(pitch_cam, 0, yaw_cam)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            # Enviar la pose al servicio de Gazebo
            req = SetModelStateRequest()
            req.model_state.model_name = "ground_observer_camera"
            req.model_state.pose = pose
            req.model_state.reference_frame = "world"
            try:
                self.set_state(req)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Error al orientar cámara: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = DroneGUI()
    gui.show()
    sys.exit(app.exec_())