from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QSizePolicy, QGroupBox, QDoubleSpinBox, QGridLayout
)
from PyQt5.QtCore import Qt

from styles import MAIN_STYLESHEET, TELEMETRY_BAR_STYLE, TELEMETRY_LABEL_STYLE
from ui_components import create_camera_label
from climate_panel_ui import ClimatePanelUI
from joystick_widget import JoystickWidget

class DroneGUIUI:
    def __init__(self, parent):
        self.parent = parent
        self.left_menu_buttons = {}
        self.system_buttons = {}
        self.control_buttons = {}
        self.labels = {}
        
        # Inicializar panel climático
        self.climate_panel = ClimatePanelUI(parent)
        
    def setup_ui(self):
        """Configurar toda la interfaz de usuario"""
        self.parent.setStyleSheet(MAIN_STYLESHEET)
        
        # Layout principal
        main_h_layout = QHBoxLayout(self.parent)
        main_h_layout.setContentsMargins(5, 5, 5, 5)
        main_h_layout.setSpacing(5)
        
        # Configurar menú lateral
        self.setup_left_menu(main_h_layout)
        
        # Configurar área central
        self.setup_center_area(main_h_layout)
        
        # Configurar panel derecho de clima
        self.climate_panel.setup_climate_panel(main_h_layout)
        
    def setup_left_menu(self, main_layout):
        """Configurar el menú lateral izquierdo"""
        self.left_menu_frame = QFrame()
        self.left_menu_frame.setMinimumWidth(50)
        self.left_menu_frame.setMaximumWidth(200)
        self.left_menu_frame.setStyleSheet("background-color: #222; border-radius: 5px; border: 1px solid #444;")
        left_menu_layout = QVBoxLayout(self.left_menu_frame)
        left_menu_layout.setAlignment(Qt.AlignTop)
        left_menu_layout.setSpacing(10)
        
        # Botones del menú izquierdo
        menu_buttons = [
            ("🚁", "Feet", "feet"),
            ("🌤️", "Climático", "climatico"),
            ("⚙️", "Ajustes", "ajustes"),
            ("📊", "Telemetría", "telemetria")
        ]
        
        for icon, text, key in menu_buttons:
            btn = QPushButton(f"{icon} {text}")
            btn.setMinimumHeight(40)
            self.left_menu_buttons[key] = btn
            left_menu_layout.addWidget(btn)
        
        # Botones de sistema
        system_buttons = [
            ("🚀", "ROS", "ros"),
            ("✈️", "ArduPilot", "ardupilot"),
            ("📡", "Telemetría", "telemetria"),
            ("🔄", "Reset World", "reset_world"),
            ("🛰️", "QGroundControl", "qgroundcontrol"),  # NUEVO BOTÓN
            ("❌", "Cerrar", "cerrar")
        ]
        
        left_menu_layout.addStretch()
        
        for icon, text, key in system_buttons:
            btn = QPushButton(f"{icon} {text}")
            btn.setMinimumHeight(35)
            self.system_buttons[key] = btn
            left_menu_layout.addWidget(btn)
        
        main_layout.addWidget(self.left_menu_frame)
        
    def setup_center_area(self, main_layout):
        """Configurar el área central con cámaras"""
        center_widget = QWidget()
        center_layout = QVBoxLayout(center_widget)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(5)
        
        # Layout de cámaras
        cameras_layout = QHBoxLayout()
        cameras_layout.setSpacing(5)
        
        # Cámara principal (PRIMERA PERSONA - FPV)
        left_col = QVBoxLayout()
        self.label_fp = create_camera_label("Vista Primera Persona", 600, 400)
        left_col.addWidget(self.label_fp)
        
        # Cámaras secundarias (Observer y Tercera Persona)
        right_col = QVBoxLayout()
        self.label_obs = create_camera_label("Vista Observer", 400, 190)
        self.label_tp = create_camera_label("Vista Tercera Persona", 400, 190)
        right_col.addWidget(self.label_obs)
        right_col.addWidget(self.label_tp)
        
        cameras_layout.addLayout(left_col, stretch=2)
        cameras_layout.addLayout(right_col, stretch=1)
        
        center_layout.addLayout(cameras_layout)
        
        # Botón de activación de cámaras
        self.btn_cam = QPushButton("🎥 Activar Streaming")
        self.btn_cam.setMaximumHeight(30)
        self.btn_cam.setMaximumWidth(200)
        center_layout.addWidget(self.btn_cam, alignment=Qt.AlignLeft)
        
        # Layout para control de cámara y telemetría en la misma fila
        bottom_row_layout = QHBoxLayout()
        bottom_row_layout.setSpacing(10)
        
        # Control de cámara con joystick (a la derecha)
        cam_group = QGroupBox("Control Cámara Observer")
        cam_group.setMaximumWidth(300)
        cam_layout = QHBoxLayout()
        cam_layout.setAlignment(Qt.AlignRight)
        
        # Joystick para controlar la cámara
        self.joystick = JoystickWidget()
        self.joystick.setMinimumSize(150, 100)
        self.joystick.setMaximumSize(150, 100)
        
        # Label para mostrar ángulos
        angle_layout = QVBoxLayout()
        self.cam_yaw_label = QLabel("Yaw: 0°")
        self.cam_pitch_label = QLabel("Pitch: 0°")
        self.cam_reset_btn = QPushButton("↺ Reset")
        self.cam_reset_btn.setMinimumHeight(25)
        self.cam_reset_btn.setMaximumWidth(80)
        
        self.cam_yaw_label.setStyleSheet("font-size: 10pt;")
        self.cam_pitch_label.setStyleSheet("font-size: 10pt;")
        
        angle_layout.addWidget(self.cam_yaw_label)
        angle_layout.addWidget(self.cam_pitch_label)
        angle_layout.addWidget(self.cam_reset_btn)
        angle_layout.addStretch()
        
        cam_layout.addWidget(self.joystick)
        cam_layout.addLayout(angle_layout)
        cam_group.setLayout(cam_layout)
        
        # Barra de telemetría (se expandirá para llenar el espacio)
        self.telemetry_bar = self.create_telemetry_bar()
        
        # Añadir a la fila inferior (telemetría a la izquierda, joystick a la derecha)
        bottom_row_layout.addWidget(self.telemetry_bar, stretch=3)
        bottom_row_layout.addWidget(cam_group, stretch=1)
        
        center_layout.addLayout(bottom_row_layout)
        
        main_layout.addWidget(center_widget, stretch=1)
        
    def create_telemetry_bar(self):
        navbar = QFrame()
        navbar.setMinimumHeight(60)
        navbar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        navbar.setStyleSheet(TELEMETRY_BAR_STYLE)
        
        # Usar GridLayout para mejor organización en espacio reducido
        telemetry_grid = QGridLayout()
        telemetry_grid.setSpacing(5)
        telemetry_grid.setContentsMargins(10, 5, 10, 5)
        
        self.labels = {
            'alt': QLabel("📏 Alt: -- m"),
            'batt': QLabel("🔋 Batt: -- %"),
            'speed': QLabel("💨 Vel: -- m/s"),
            'mode': QLabel("🚀 Modo: -- | Arm: --"),
            'voltage': QLabel("⚡ Volt: -- V"),
            'current': QLabel("🔌 Curr: -- A"),
            'gps': QLabel("📍 GPS: --, --"),
            'heading': QLabel("🧭 Head: --°")
        }
        
        # Configurar todos los labels
        for lbl in self.labels.values():
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(TELEMETRY_LABEL_STYLE + "font-size: 10pt; padding: 2px;")
            lbl.setMinimumWidth(90)
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        
        # Organizar en grid 2x4 para mejor uso del espacio
        telemetry_grid.addWidget(self.labels['alt'], 0, 0)
        telemetry_grid.addWidget(self.labels['batt'], 0, 1)
        telemetry_grid.addWidget(self.labels['speed'], 0, 2)
        telemetry_grid.addWidget(self.labels['mode'], 0, 3)
        telemetry_grid.addWidget(self.labels['voltage'], 1, 0)
        telemetry_grid.addWidget(self.labels['current'], 1, 1)
        telemetry_grid.addWidget(self.labels['gps'], 1, 2)
        telemetry_grid.addWidget(self.labels['heading'], 1, 3)
        
        navbar.setLayout(telemetry_grid)
        return navbar