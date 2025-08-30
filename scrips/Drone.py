#!/usr/bin/env python3
import sys
import os
import rospy
import cv2
import subprocess
from threading import Thread
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QPixmap, QImage, QIcon

class DroneGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador EMI - Control de Dron")
        self.setGeometry(100, 100, 700, 900)
        self.setWindowIcon(QIcon("/home/niwre21/catkin_ws/src/emi_trop/scripts/dronicoono.png"))

        self.bridge = CvBridge()
        self.image_fp = None
        self.image_tp = None

        self.altitude = 0.0
        self.battery = 0.0
        self.speed = 0.0
        self.mode = "DESCONOCIDO"
        self.armed = False

        self.sub_fp = None
        self.sub_tp = None
        self.sub_alt = None
        self.sub_batt = None
        self.sub_speed = None
        self.sub_state = None

        self.menu_expanded = False
        self.menu_animation = None

        # --- Inicia roscore automáticamente ---
        self.start_roscore()

        self.init_ui()
        self.init_ros()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_images)
        self.timer.start(50)

    # === Iniciar roscore en segundo plano ===
    def start_roscore(self):
        def run_roscore():
            subprocess.Popen(
                ["roscore"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setpgrp  # Permite que roscore corra independiente
            )
        Thread(target=run_roscore, daemon=True).start()

    # === Interfaz ===
    def init_ui(self):
        self.setStyleSheet("""
            QWidget {background: #1b1b1b; color: #eee; font-family: 'Segoe UI', Tahoma;}
            QLabel#cameraLabel {font-size: 16px; color: #bbb;}
        """)

        main_layout = QVBoxLayout(self)
        main_h_layout = QHBoxLayout()

        # Menú lateral
        self.menu_frame = QFrame()
        self.menu_frame.setMinimumWidth(30)
        self.menu_frame.setMaximumWidth(200)
        self.menu_frame.setStyleSheet("background-color: #009999;")
        self.menu_layout = QVBoxLayout(self.menu_frame)
        self.menu_layout.setAlignment(Qt.AlignTop)

        button_style = """
        QPushButton {
            background-color: #00CCCC;
            color: #000;
            border: 1px solid #A0A0A0;
            border-radius: 5px;
            padding: 8px 12px;
            font-weight: bold;
        }
        QPushButton:hover {background-color: #66DDDD;}
        QPushButton:pressed {background-color: #44BBBB;}
        """

        self.btn_toggle_menu = QPushButton("☰")
        self.btn_toggle_menu.setStyleSheet(button_style)
        self.btn_toggle_menu.clicked.connect(self.toggle_menu)
        self.menu_layout.addWidget(self.btn_toggle_menu)

        # Botones simulación
        self.btn_launch_ros = QPushButton("ROS Launch")
        self.btn_launch_ros.setStyleSheet(button_style)
        self.btn_launch_ros.clicked.connect(self.run_ros_launch)
        self.btn_launch_ros.setVisible(False)

        self.btn_launch_ardupilot = QPushButton("ArduPilot SITL")
        self.btn_launch_ardupilot.setStyleSheet(button_style)
        self.btn_launch_ardupilot.clicked.connect(self.run_ardupilot)
        self.btn_launch_ardupilot.setVisible(False)

        self.btn_launch_apm = QPushButton("Telemetría APM")
        self.btn_launch_apm.setStyleSheet(button_style)
        self.btn_launch_apm.clicked.connect(self.run_apm_launch)
        self.btn_launch_apm.setVisible(False)

        self.btn_close_server = QPushButton("Cerrar Servidor")
        self.btn_close_server.setStyleSheet(button_style)
        self.btn_close_server.clicked.connect(self.close_gazebo)
        self.btn_close_server.setVisible(False)

        for btn in [self.btn_launch_ros, self.btn_launch_ardupilot, self.btn_launch_apm, self.btn_close_server]:
            self.menu_layout.addWidget(btn)

        main_h_layout.addWidget(self.menu_frame)

        # Cámaras
        camera_layout = QVBoxLayout()
        self.btn_cam = QPushButton("Activar cámaras")
        self.btn_cam.setStyleSheet(button_style)
        self.btn_cam.clicked.connect(self.toggle_subs)
        camera_layout.addWidget(self.btn_cam)

        self.label_fp = QLabel("SIN SEÑAL (Primera Persona)")
        self.label_fp.setObjectName("cameraLabel")
        self.label_fp.setAlignment(Qt.AlignCenter)
        self.label_fp.setFrameShape(QFrame.Box)
        self.label_fp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label_fp.setMinimumSize(200, 150)

        self.label_tp = QLabel("SIN SEÑAL (Tercera Persona)")
        self.label_tp.setObjectName("cameraLabel")
        self.label_tp.setAlignment(Qt.AlignCenter)
        self.label_tp.setFrameShape(QFrame.Box)
        self.label_tp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label_tp.setMinimumSize(200, 150)

        camera_layout.addWidget(self.label_fp, stretch=1)
        camera_layout.addWidget(self.label_tp, stretch=1)

        main_h_layout.addLayout(camera_layout, stretch=1)
        main_layout.addLayout(main_h_layout)

        # Telemetría
        navbar = QFrame()
        navbar.setFixedHeight(50)
        navbar.setStyleSheet("background-color: #222; color: white;")
        telemetry_layout = QHBoxLayout()
        self.label_alt = QLabel("Altitud: -- m")
        self.label_batt = QLabel("Batería: -- %")
        self.label_speed = QLabel("Velocidad: -- m/s")
        self.label_mode = QLabel("Modo: -- | Armado: --")
        for lbl in [self.label_alt, self.label_batt, self.label_speed, self.label_mode]:
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("font-size: 16px; padding: 5px;")
            telemetry_layout.addWidget(lbl)
        navbar.setLayout(telemetry_layout)
        main_layout.addWidget(navbar)

    def toggle_menu(self):
        if self.menu_animation and self.menu_animation.state() == QPropertyAnimation.Running:
            return
        start_width = self.menu_frame.width()
        end_width = 200 if not self.menu_expanded else 30
        self.menu_animation = QPropertyAnimation(self.menu_frame, b"minimumWidth")
        self.menu_animation.setDuration(300)
        self.menu_animation.setStartValue(start_width)
        self.menu_animation.setEndValue(end_width)
        self.menu_animation.setEasingCurve(QEasingCurve.InOutQuad)
        def on_finished():
            self.menu_expanded = not self.menu_expanded
            for btn in [self.btn_launch_ros, self.btn_launch_ardupilot, self.btn_launch_apm, self.btn_close_server]:
                btn.setVisible(self.menu_expanded)
        self.menu_animation.finished.connect(on_finished)
        self.menu_animation.start()

    # === ROS ===
    def init_ros(self):
        rospy.init_node('drone_gui_pyqt', anonymous=True, disable_signals=True)

    def toggle_subs(self):
        if not self.sub_fp:
            self.sub_fp = rospy.Subscriber("/iris/rear_follow_cam/rear_follow_cam/image_raw", Image, self.cb_first_person)
            self.sub_tp = rospy.Subscriber("/webcam/image_raw", Image, self.cb_third_person)
            self.sub_alt = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_pose)
            self.sub_batt = rospy.Subscriber("/mavros/battery", BatteryState, self.cb_battery)
            self.sub_speed = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_velocity)
            self.sub_state = rospy.Subscriber("/mavros/state", State, self.cb_state)
            self.btn_cam.setText("Desactivar cámaras")
        else:
            for sub in [self.sub_fp, self.sub_tp, self.sub_alt, self.sub_batt, self.sub_speed, self.sub_state]:
                if sub: sub.unregister()
            self.sub_fp = self.sub_tp = self.sub_alt = self.sub_batt = None
            self.sub_speed = self.sub_state = None
            self.btn_cam.setText("Activar cámaras")
            self.label_fp.setText("SIN SEÑAL (Primera Persona)")
            self.label_tp.setText("SIN SEÑAL (Tercera Persona)")
            self.label_alt.setText("Altitud: -- m")
            self.label_batt.setText("Batería: -- %")
            self.label_speed.setText("Velocidad: -- m/s")
            self.label_mode.setText("Modo: -- | Armado: --")

    # === Callbacks ===
    def cb_first_person(self, data):
        try: self.image_fp = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e: rospy.logwarn(f"FP Error: {e}")

    def cb_third_person(self, data):
        try: self.image_tp = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e: rospy.logwarn(f"TP Error: {e}")

    def cb_pose(self, msg):
        self.altitude = msg.pose.position.z
        self.label_alt.setText(f"Altitud: {self.altitude:.2f} m")

    def cb_battery(self, msg):
        self.label_batt.setText(f"Batería: {msg.percentage*100:.0f} %")

    def cb_velocity(self, msg):
        vx = msg.pose.position.x
        vy = msg.pose.position.y
        vz = msg.pose.position.z
        speed = (vx**2 + vy**2 + vz**2)**0.5
        self.label_speed.setText(f"Velocidad: {speed:.2f} m/s")

    def cb_state(self, msg):
        self.mode = msg.mode
        self.armed = msg.armed
        self.label_mode.setText(f"Modo: {self.mode} | Armado: {self.armed}")

    # === Utils ===
    def cv_to_pixmap_responsive(self, cv_img, label):
        if cv_img is None: return
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch*w
        qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        scaled = qt_img.scaled(label.width(), label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        return QPixmap.fromImage(scaled)

    def refresh_images(self):
        if self.image_fp is not None: self.label_fp.setPixmap(self.cv_to_pixmap_responsive(self.image_fp, self.label_fp))
        if self.image_tp is not None: self.label_tp.setPixmap(self.cv_to_pixmap_responsive(self.image_tp, self.label_tp))

    # === Funciones simulación ===
    def run_ros_launch(self):
        Thread(target=self._run_ros_launch_thread, daemon=True).start()
    def _run_ros_launch_thread(self):
        subprocess.Popen(["roslaunch", "emi_trop", "emilaunch.launch"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run_ardupilot(self):
        Thread(target=self._run_ardupilot_thread, daemon=True).start()
    def _run_ardupilot_thread(self):
        subprocess.Popen(["bash","-c","cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --location emi"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run_apm_launch(self):
        Thread(target=self._run_apm_thread, daemon=True).start()
    def _run_apm_thread(self):
        subprocess.Popen(["roslaunch", "emi_trop", "apm.launch"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def close_gazebo(self):
        Thread(target=self._close_gazebo_thread, daemon=True).start()
    def _close_gazebo_thread(self):
        subprocess.Popen(["killall", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.Popen(["killall", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = DroneGUI()
    gui.show()
    sys.exit(app.exec_())
