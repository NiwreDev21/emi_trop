# climate_panel_ui.py
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QDoubleSpinBox, QSlider
)
from PyQt5.QtCore import Qt
from ui_components import create_slider_control, create_spinbox_control, create_control_group

class ClimatePanelUI:
    def __init__(self, parent):
        self.parent = parent
        self.ambient_sliders = {}
        self.cloud_controls = {}
        self.wind_controls = {}
        self.turb_controls = {}
        
    def setup_climate_panel(self, main_layout):
        """Configurar el panel lateral derecho de clima"""
        from styles import MAIN_STYLESHEET
        
        self.right_panel_frame = QWidget()
        self.right_panel_frame.setMinimumWidth(50)
        self.right_panel_frame.setMaximumWidth(400)
        self.right_panel_frame.setStyleSheet("background-color: #2B2B2B; border-radius: 5px;")
        self.right_panel_frame.hide()
        
        right_panel_layout = QVBoxLayout(self.right_panel_frame)
        right_panel_layout.setContentsMargins(10, 10, 10, 10)
        right_panel_layout.setSpacing(10)
        
        # Título del panel
        title_label = QLabel("🌤️ CONTROL CLIMÁTICO")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #3498db; padding: 10px;")
        title_label.setAlignment(Qt.AlignCenter)
        right_panel_layout.addWidget(title_label)
        
        # Controles climáticos
        self.init_climate_controls(right_panel_layout)
        
        right_panel_layout.addStretch()
        
        # Botones de acción
        action_layout = QHBoxLayout()
        self.apply_btn = QPushButton("✅ Aplicar")
        self.reset_btn = QPushButton("🔄 Reset")
        
        action_layout.addWidget(self.apply_btn)
        action_layout.addWidget(self.reset_btn)
        right_panel_layout.addLayout(action_layout)
        
        main_layout.addWidget(self.right_panel_frame)
        
    def init_climate_controls(self, layout):
        # Luz ambiental
        ambient_layout = QVBoxLayout()
        for color in ["Rojo", "Verde", "Azul"]:
            color_layout, slider, value_label = create_slider_control(color, 0, 100, 60)
            ambient_layout.addLayout(color_layout)
            self.ambient_sliders[color.lower()] = slider
        
        ambient_group = create_control_group("💡 Luz Ambiental", ambient_layout)
        layout.addWidget(ambient_group)
        
        # Nubes
        clouds_layout = QVBoxLayout()
        cloud_params = [
            ("Velocidad", 0, 20, 5, 1),
            ("Tamaño", 0.1, 5, 0.4, 0.1),
            ("Humedad", 0, 1, 0.8, 0.05)
        ]
        
        for name, min_val, max_val, default, step in cloud_params:
            param_layout, spinbox = create_spinbox_control(name, min_val, max_val, default, step)
            clouds_layout.addLayout(param_layout)
            self.cloud_controls[name.lower()] = spinbox
        
        clouds_group = create_control_group("☁️ Nubes", clouds_layout)
        layout.addWidget(clouds_group)
        
        # Viento
        wind_layout = QVBoxLayout()
        for axis in ["X", "Y", "Z"]:
            param_layout, spinbox = create_spinbox_control(f"Velocidad {axis}", -50, 50, 0, 1, 1)
            wind_layout.addLayout(param_layout)
            self.wind_controls[axis.lower()] = spinbox
        
        wind_group = create_control_group("💨 Viento", wind_layout)
        layout.addWidget(wind_group)
        
        # Turbulencia
        turb_layout = QVBoxLayout()
        for axis in ["X", "Y", "Z"]:
            param_layout, spinbox = create_spinbox_control(f"Turb {axis}", 0, 50, 0, 1, 1)
            turb_layout.addLayout(param_layout)
            self.turb_controls[axis.lower()] = spinbox
        
        turb_group = create_control_group("🌪️ Turbulencia", turb_layout)
        layout.addWidget(turb_group)