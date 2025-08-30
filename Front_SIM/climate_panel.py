#climate_panel.py
import rospy
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSlider,
    QHBoxLayout, QPushButton, QDoubleSpinBox, QGroupBox
)
from PyQt5.QtCore import Qt

class ClimateControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel de Control Climático - Gazebo")
        self.setGeometry(100, 100, 500, 800)
        
        # Inicializar la interfaz directamente en el constructor
        self.setStyleSheet("""
            QWidget {
                background: #2c3e50;
                color: #ecf0f1;
                font-family: 'Segoe UI';
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #34495e;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
                background: #34495e;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #3498db;
            }
            QLabel {
                color: #bdc3c7;
                padding: 5px;
            }
            QSlider::groove:horizontal {
                border: 1px solid #34495e;
                height: 8px;
                background: #34495e;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #3498db;
                border: 1px solid #2980b9;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QDoubleSpinBox {
                background: #34495e;
                border: 1px solid #2c3e50;
                color: #ecf0f1;
                padding: 5px;
                border-radius: 4px;
            }
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3498db, stop:1 #2980b9);
                border: 1px solid #2980b9;
                color: white;
                padding: 10px;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2980b9, stop:1 #3498db);
            }
            QPushButton:pressed {
                background: #2c3e50;
            }
        """)
        
        main_layout = QVBoxLayout(self)
        
        # --- Luz Ambiental ---
        ambient_group = QGroupBox("Luz Ambiental (RGBA)")
        ambient_layout = QVBoxLayout()
        
        self.ambient_sliders = []
        colors = ["Rojo", "Verde", "Azul", "Alpha"]
        initial_values = [60, 60, 60, 50]
        
        for i, color in enumerate(colors):
            color_layout = QHBoxLayout()
            lbl = QLabel(f"{color}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(initial_values[i])
            value_label = QLabel(f"{initial_values[i]/100:.1f}")
            
            slider.valueChanged.connect(
                lambda value, lbl=value_label: lbl.setText(f"{value/100:.1f}")
            )
            
            color_layout.addWidget(lbl)
            color_layout.addWidget(slider)
            color_layout.addWidget(value_label)
            ambient_layout.addLayout(color_layout)
            self.ambient_sliders.append(slider)
        
        ambient_group.setLayout(ambient_layout)
        main_layout.addWidget(ambient_group)
        
        # --- Color de Fondo ---
        bg_group = QGroupBox("Color de Fondo (RGBA)")
        bg_layout = QVBoxLayout()
        
        self.bg_sliders = []
        bg_initial_values = [80, 80, 80, 100]
        
        for i, color in enumerate(colors):
            color_layout = QHBoxLayout()
            lbl = QLabel(f"{color}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(bg_initial_values[i])
            value_label = QLabel(f"{bg_initial_values[i]/100:.1f}")
            
            slider.valueChanged.connect(
                lambda value, lbl=value_label: lbl.setText(f"{value/100:.1f}")
            )
            
            color_layout.addWidget(lbl)
            color_layout.addWidget(slider)
            color_layout.addWidget(value_label)
            bg_layout.addLayout(color_layout)
            self.bg_sliders.append(slider)
        
        bg_group.setLayout(bg_layout)
        main_layout.addWidget(bg_group)
        
        # --- Nubes ---
        clouds_group = QGroupBox("Configuración de Nubes")
        clouds_layout = QVBoxLayout()
        
        clouds_params = [
            ("Velocidad de nubes", "cloud_speed", 0, 50, 5, 1),
            ("Dirección X", "cloud_dir_x", -10, 10, 0.5, 0.1),
            ("Dirección Y", "cloud_dir_y", -10, 10, 0, 0.1),
            ("Tamaño medio", "cloud_size", 0.1, 5, 0.4, 0.1),
            ("Humedad", "cloud_humidity", 0, 1, 0.8, 0.05)
        ]
        
        self.cloud_controls = {}
        
        for param_name, param_key, min_val, max_val, default, step in clouds_params:
            param_layout = QHBoxLayout()
            lbl = QLabel(param_name)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default)
            spinbox.setSingleStep(step)
            spinbox.setDecimals(2)
            
            param_layout.addWidget(lbl)
            param_layout.addWidget(spinbox)
            clouds_layout.addLayout(param_layout)
            self.cloud_controls[param_key] = spinbox
        
        clouds_group.setLayout(clouds_layout)
        main_layout.addWidget(clouds_group)
        
        # --- Viento ---
        wind_group = QGroupBox("Configuración de Viento")
        wind_layout = QVBoxLayout()
        
        wind_params = [
            ("Velocidad X", "wind_x", -50, 50, 3, 1),
            ("Velocidad Y", "wind_y", -50, 50, 0, 1),
            ("Velocidad Z", "wind_z", -50, 50, 0, 1)
        ]
        
        self.wind_controls = {}
        
        for param_name, param_key, min_val, max_val, default, step in wind_params:
            param_layout = QHBoxLayout()
            lbl = QLabel(param_name)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default)
            spinbox.setSingleStep(step)
            spinbox.setDecimals(1)
            
            param_layout.addWidget(lbl)
            param_layout.addWidget(spinbox)
            wind_layout.addLayout(param_layout)
            self.wind_controls[param_key] = spinbox
        
        wind_group.setLayout(wind_layout)
        main_layout.addWidget(wind_group)
        
        # --- Turbulencia ---
        turb_group = QGroupBox("Configuración de Turbulencia")
        turb_layout = QVBoxLayout()
        
        turb_params = [
            ("Turbulencia X", "turb_x", 0, 50, 7, 1),
            ("Turbulencia Y", "turb_y", 0, 50, 0, 1),
            ("Turbulencia Z", "turb_z", 0, 50, 0, 1)
        ]
        
        self.turb_controls = {}
        
        for param_name, param_key, min_val, max_val, default, step in turb_params:
            param_layout = QHBoxLayout()
            lbl = QLabel(param_name)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default)
            spinbox.setSingleStep(step)
            spinbox.setDecimals(1)
            
            param_layout.addWidget(lbl)
            param_layout.addWidget(spinbox)
            turb_layout.addLayout(param_layout)
            self.turb_controls[param_key] = spinbox
        
        turb_group.setLayout(turb_layout)
        main_layout.addWidget(turb_group)
        
        # --- Botones ---
        button_layout = QHBoxLayout()
        
        self.update_btn = QPushButton("🌤️ Aplicar Cambios")
        self.update_btn.clicked.connect(self.update_climate)
        
        self.reset_btn = QPushButton("🔄 Restablecer")
        self.reset_btn.clicked.connect(self.reset_values)
        
        button_layout.addWidget(self.update_btn)
        button_layout.addWidget(self.reset_btn)
        main_layout.addLayout(button_layout)
        
    def update_climate(self):
        # Obtener valores actuales
        ambient = [s.value()/100.0 for s in self.ambient_sliders]
        background = [s.value()/100.0 for s in self.bg_sliders]
        
        clouds = {
            "speed": self.cloud_controls["cloud_speed"].value(),
            "direction_x": self.cloud_controls["cloud_dir_x"].value(),
            "direction_y": self.cloud_controls["cloud_dir_y"].value(),
            "size": self.cloud_controls["cloud_size"].value(),
            "humidity": self.cloud_controls["cloud_humidity"].value()
        }
        
        wind = [
            self.wind_controls["wind_x"].value(),
            self.wind_controls["wind_y"].value(),
            self.wind_controls["wind_z"].value()
        ]
        
        turbulence = [
            self.turb_controls["turb_x"].value(),
            self.turb_controls["turb_y"].value(),
            self.turb_controls["turb_z"].value()
        ]
        
        # Aquí iría la conexión real con Gazebo
        print("=== ACTUALIZANDO CONFIGURACIÓN CLIMÁTICA ===")
        print(f"Luz Ambiental: R={ambient[0]:.2f} G={ambient[1]:.2f} B={ambient[2]:.2f} A={ambient[3]:.2f}")
        print(f"Fondo: R={background[0]:.2f} G={background[1]:.2f} B={background[2]:.2f} A={background[3]:.2f}")
        print(f"Nubes: Velocidad={clouds['speed']} Dirección=({clouds['direction_x']}, {clouds['direction_y']})")
        print(f"       Tamaño={clouds['size']} Humedad={clouds['humidity']}")
        print(f"Viento: X={wind[0]} Y={wind[1]} Z={wind[2]}")
        print(f"Turbulencia: X={turbulence[0]} Y={turbulence[1]} Z={turbulence[2]}")
        print("============================================")
        
        # TODO: Implementar ROS services para Gazebo
        # self.apply_climate_settings(ambient, background, clouds, wind, turbulence)
        
    def reset_values(self):
        # Restablecer valores por defecto
        default_ambient = [60, 60, 60, 50]
        default_bg = [80, 80, 80, 100]
        
        for i, slider in enumerate(self.ambient_sliders):
            slider.setValue(default_ambient[i])
        
        for i, slider in enumerate(self.bg_sliders):
            slider.setValue(default_bg[i])
        
        self.cloud_controls["cloud_speed"].setValue(5)
        self.cloud_controls["cloud_dir_x"].setValue(0.5)
        self.cloud_controls["cloud_dir_y"].setValue(0)
        self.cloud_controls["cloud_size"].setValue(0.4)
        self.cloud_controls["cloud_humidity"].setValue(0.8)
        
        self.wind_controls["wind_x"].setValue(3)
        self.wind_controls["wind_y"].setValue(0)
        self.wind_controls["wind_z"].setValue(0)
        
        self.turb_controls["turb_x"].setValue(7)
        self.turb_controls["turb_y"].setValue(0)
        self.turb_controls["turb_z"].setValue(0)
        
        print("Valores restablecidos a configuración por defecto")