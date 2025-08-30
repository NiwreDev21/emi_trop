#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QSlider,
    QHBoxLayout, QPushButton, QDoubleSpinBox
)
from PyQt5.QtCore import Qt

# Servicios o publishers de Gazebo
# Para este ejemplo, se simula la actualización.
# En un entorno real, usarías ROS services como:
# /gazebo/set_light_properties, /gazebo/set_model_state, o un plugin custom

class ClimateControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel de control climático - Gazebo")
        self.setGeometry(100, 100, 500, 600)

        # Layout principal
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # --- Luz ambiental ---
        main_layout.addWidget(QLabel("Luz ambiental (RGBA)"))
        self.ambient_sliders = []
        ambient_layout = QHBoxLayout()
        for i, color in enumerate(["R","G","B","A"]):
            lbl = QLabel(color)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(60 if i<3 else 50)  # valores iniciales: 0.6,0.6,0.5,0.5
            ambient_layout.addWidget(lbl)
            ambient_layout.addWidget(slider)
            self.ambient_sliders.append(slider)
        main_layout.addLayout(ambient_layout)

        # --- Color de fondo ---
        main_layout.addWidget(QLabel("Color de fondo (RGBA)"))
        self.bg_sliders = []
        bg_layout = QHBoxLayout()
        for i, color in enumerate(["R","G","B","A"]):
            lbl = QLabel(color)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100)
            slider.setValue(80 if i<3 else 100)
            bg_layout.addWidget(lbl)
            bg_layout.addWidget(slider)
            self.bg_sliders.append(slider)
        main_layout.addLayout(bg_layout)

        # --- Nubes ---
        main_layout.addWidget(QLabel("Velocidad de nubes"))
        self.cloud_speed = QDoubleSpinBox()
        self.cloud_speed.setRange(0, 50)
        self.cloud_speed.setValue(5)
        main_layout.addWidget(self.cloud_speed)

        main_layout.addWidget(QLabel("Dirección de nubes X"))
        self.cloud_dir_x = QDoubleSpinBox()
        self.cloud_dir_x.setRange(-10, 10)
        self.cloud_dir_x.setValue(0.5)
        main_layout.addWidget(self.cloud_dir_x)

        main_layout.addWidget(QLabel("Dirección de nubes Y"))
        self.cloud_dir_y = QDoubleSpinBox()
        self.cloud_dir_y.setRange(-10, 10)
        self.cloud_dir_y.setValue(0)
        main_layout.addWidget(self.cloud_dir_y)

        main_layout.addWidget(QLabel("Tamaño medio de nubes"))
        self.cloud_size = QDoubleSpinBox()
        self.cloud_size.setRange(0.1, 5)
        self.cloud_size.setValue(0.4)
        main_layout.addWidget(self.cloud_size)

        main_layout.addWidget(QLabel("Humedad nubes"))
        self.cloud_humidity = QDoubleSpinBox()
        self.cloud_humidity.setRange(0,1)
        self.cloud_humidity.setSingleStep(0.05)
        self.cloud_humidity.setValue(0.8)
        main_layout.addWidget(self.cloud_humidity)

        # --- Viento ---
        main_layout.addWidget(QLabel("Velocidad de viento X"))
        self.wind_x = QDoubleSpinBox()
        self.wind_x.setRange(-50,50)
        self.wind_x.setValue(3)
        main_layout.addWidget(self.wind_x)

        main_layout.addWidget(QLabel("Velocidad de viento Y"))
        self.wind_y = QDoubleSpinBox()
        self.wind_y.setRange(-50,50)
        self.wind_y.setValue(0)
        main_layout.addWidget(self.wind_y)

        main_layout.addWidget(QLabel("Velocidad de viento Z"))
        self.wind_z = QDoubleSpinBox()
        self.wind_z.setRange(-50,50)
        self.wind_z.setValue(0)
        main_layout.addWidget(self.wind_z)

        # --- Turbulencia ---
        main_layout.addWidget(QLabel("Turbulencia X"))
        self.turb_x = QDoubleSpinBox()
        self.turb_x.setRange(0,50)
        self.turb_x.setValue(7)
        main_layout.addWidget(self.turb_x)

        main_layout.addWidget(QLabel("Turbulencia Y"))
        self.turb_y = QDoubleSpinBox()
        self.turb_y.setRange(0,50)
        self.turb_y.setValue(0)
        main_layout.addWidget(self.turb_y)

        main_layout.addWidget(QLabel("Turbulencia Z"))
        self.turb_z = QDoubleSpinBox()
        self.turb_z.setRange(0,50)
        self.turb_z.setValue(0)
        main_layout.addWidget(self.turb_z)

        # --- Botón actualizar ---
        self.update_btn = QPushButton("Actualizar cambios en Gazebo")
        self.update_btn.clicked.connect(self.update_climate)
        main_layout.addWidget(self.update_btn)

    def update_climate(self):
        # Aquí se conectarían los servicios ROS o topics para aplicar cambios reales en Gazebo
        ambient = [s.value()/100.0 for s in self.ambient_sliders]
        background = [s.value()/100.0 for s in self.bg_sliders]
        clouds = {
            "speed": self.cloud_speed.value(),
            "direction": [self.cloud_dir_x.value(), self.cloud_dir_y.value()],
            "size": self.cloud_size.value(),
            "humidity": self.cloud_humidity.value()
        }
        wind = [self.wind_x.value(), self.wind_y.value(), self.wind_z.value()]
        turbulence = [self.turb_x.value(), self.turb_y.value(), self.turb_z.value()]

        print("=== Actualizando clima ===")
        print("Ambient:", ambient)
        print("Background:", background)
        print("Clouds:", clouds)
        print("Wind:", wind)
        print("Turbulence:", turbulence)
        print("==========================")

        # TODO: aquí se llamarían ROS services o topics
        # Ejemplo: self.set_wind_srv(wind_vector), self.set_clouds_srv(clouds), etc.


if __name__ == "__main__":
    rospy.init_node('climate_control_panel', anonymous=True)
    app = QApplication(sys.argv)
    panel = ClimateControlPanel()
    panel.show()
    sys.exit(app.exec_())
