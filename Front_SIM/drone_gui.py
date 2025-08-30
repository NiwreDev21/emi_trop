import sys
import os
import rospy
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QSizePolicy, QSlider, QTabWidget, QMessageBox,
    QScrollArea, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QIcon

from ros_manager import ROSManager
from utils import cv2_to_pixmap, launch_ros, launch_ardupilot, launch_apm, close_gazebo, launch_qgroundcontrol
from config import WINDOW_TITLE, WINDOW_SIZE
from drone_gui_ui import DroneGUIUI

class DroneGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(WINDOW_TITLE)
        self.resize(*WINDOW_SIZE)
        
        self.ros_manager = ROSManager()
        if not self.ros_manager.init_ros():
            print("Advertencia: ROS Manager no pudo inicializarse correctamente")
        
        self.subscriptions_active = False
        self.left_menu_expanded = False
        self.right_panel_expanded = False
        self.mavproxy_running = False
        self.map_visible = True
        
        self.ui = DroneGUIUI(self)
        self.ui.setup_ui()
        
        self._connect_signals()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)
        
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.auto_focus_camera)
        self.camera_timer.start(50)
        
        # Timer para actualizar el mapa
        self.map_timer = QTimer()
        self.map_timer.timeout.connect(self.update_map_display)
        self.map_timer.start(1000)  # Actualizar cada segundo
        
        self.setMinimumSize(1400, 900)
        
    def _connect_signals(self):
        """Conectar todas las señales de la interfaz"""
        self.ui.left_menu_buttons['feet'].clicked.connect(self.show_drone_view)
        self.ui.left_menu_buttons['climatico'].clicked.connect(self.toggle_climate_panel)
        
        self.ui.joystick.angleChanged.connect(self.rotate_observer_camera)
        self.ui.joystick.mousePressed.connect(self.start_manual_control)
        self.ui.joystick.mouseReleased.connect(self.end_manual_control)
        self.ui.cam_reset_btn.clicked.connect(self.reset_camera)
        
        self.ui.system_buttons['ros'].clicked.connect(launch_ros)
        self.ui.system_buttons['ardupilot'].clicked.connect(launch_ardupilot)
        self.ui.system_buttons['telemetria'].clicked.connect(launch_apm)
        self.ui.system_buttons['mavproxy'].clicked.connect(self.toggle_mavproxy)
        self.ui.system_buttons['reset_world'].clicked.connect(self.reset_world)
        self.ui.system_buttons['qgroundcontrol'].clicked.connect(self.launch_qgc)
        self.ui.system_buttons['cerrar'].clicked.connect(close_gazebo)
        
        self.ui.btn_cam.clicked.connect(self.toggle_cameras)
        self.ui.btn_refresh_map.clicked.connect(self.refresh_map)
        self.ui.btn_toggle_map.clicked.connect(self.toggle_map_visibility)
        
        self.ui.climate_panel.apply_btn.clicked.connect(self.apply_climate_settings)
        self.ui.climate_panel.reset_btn.clicked.connect(self.reset_climate_settings)
        
    def toggle_mavproxy(self):
        """Activar/desactivar visualización de mapa MAVProxy"""
        self.mavproxy_running = not self.mavproxy_running
        
        if self.mavproxy_running:
            self.ui.map_status.setText("Estado: Simulando")
            self.ui.map_status.setStyleSheet("color: #66ff66; font-size: 10px;")
            self.show_message("🗺️ Mapa MAVProxy", "Visualización de mapa activada")
        else:
            self.ui.map_status.setText("Estado: Inactivo")
            self.ui.map_status.setStyleSheet("color: #ff6666; font-size: 10px;")
            self.ui.map_label.setText("Mapa MAVProxy\n---\nDesactivado")
            self.show_message("🗺️ Mapa MAVProxy", "Visualización de mapa desactivada")
            
    def refresh_map(self):
        """Refrescar la visualización del mapa"""
        if self.mavproxy_running:
            self.ui.map_label.setText("Actualizando mapa...\nEspere por favor")
            self.show_message("🗺️ Mapa", "Actualizando visualización del mapa")
        else:
            self.show_message("❌ Error", "MAVProxy no está activo")
            
    def toggle_map_visibility(self):
        """Alternar visibilidad del mapa"""
        self.map_visible = not self.map_visible
        
        if self.map_visible:
            self.ui.map_label.show()
            self.ui.btn_toggle_map.setStyleSheet("")
        else:
            self.ui.map_label.hide()
            self.ui.btn_toggle_map.setStyleSheet("background-color: #444;")
            
    def update_map_display(self):
        """Actualizar la visualización del mapa con datos simulados"""
        if self.mavproxy_running and self.subscriptions_active:
            try:
                # Obtener datos de telemetría
                gps_data = self.ros_manager.telemetry_data['gps']
                altitude = self.ros_manager.telemetry_data['altitude']
                heading = self.ros_manager.telemetry_data['heading']
                
                # Crear representación visual del mapa
                map_text = "Mapa MAVProxy\n"
                map_text += "---\n"
                
                if gps_data and gps_data['lat'] != 0 and gps_data['lon'] != 0:
                    map_text += f"Lat: {gps_data['lat']:.6f}\n"
                    map_text += f"Lon: {gps_data['lon']:.6f}\n"
                    map_text += f"Alt: {altitude:.1f}m\n"
                    map_text += f"Head: {heading:.0f}°\n"
                    
                    # Representación visual simple del dron
                    map_text += "---\n"
                    map_text += "    ^ N\n"
                    map_text += "    |\n"
                    map_text += "W --+-- E\n"
                    map_text += "    |\n"
                    map_text += "    v S\n"
                    map_text += "---\n"
                    map_text += f"🛩️ Pos: ({gps_data['lat']:.4f}, {gps_data['lon']:.4f})"
                else:
                    map_text += "Esperando datos GPS...\n"
                    map_text += "Conectando con el dron"
                    
                self.ui.map_label.setText(map_text)
                
            except Exception as e:
                print(f"Error actualizando mapa: {e}")
                self.ui.map_label.setText("Mapa MAVProxy\n---\nError actualizando\nVerifique conexión")
        
    def launch_qgc(self):
        """Iniciar QGroundControl"""
        try:
            if launch_qgroundcontrol():
                self.show_message("✅ QGroundControl", "QGroundControl se está iniciando...")
            else:
                self.show_message("❌ Error", "No se pudo iniciar QGroundControl. Verifica la ruta.")
        except Exception as e:
            print(f"Error iniciando QGroundControl: {e}")
            self.show_message("❌ Error", f"Error iniciando QGroundControl: {str(e)}")
            
    def start_manual_control(self):
        """Iniciar control manual de la cámara"""
        self.ros_manager.set_manual_override(True)
        
    def end_manual_control(self):
        """Terminar control manual de la cámara"""
        self.ros_manager.set_manual_override(False)
        self.auto_focus_camera()
        
    def auto_focus_camera(self):
        """Enfocar automáticamente la cámara al dron"""
        if self.subscriptions_active and not self.ros_manager.manual_override:
            try:
                yaw = self.ros_manager.rotate_observer_camera(0)
                self.ui.cam_yaw_label.setText(f"Yaw: {yaw:.1f}°")
                self.ui.cam_pitch_label.setText("Pitch: 0.0°")
            except Exception as e:
                print(f"Error en auto-enfoque: {e}")
        
    def reset_world(self):
        """Resetear el mundo de Gazebo"""
        try:
            from std_srvs.srv import Empty
            rospy.wait_for_service('/gazebo/reset_world', timeout=5)
            reset_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_service()
            self.show_message("✅ Mundo Resetado", "El mundo de Gazebo ha sido resetado correctamente")
        except Exception as e:
            print(f"Error reseteando mundo: {e}")
            self.show_message("❌ Error", f"Error reseteando mundo: {str(e)}")
            
    def rotate_observer_camera(self, yaw):
        """Rotar la cámara observer solo horizontalmente"""
        try:
            self.ui.cam_yaw_label.setText(f"Yaw: {yaw:.1f}°")
            self.ui.cam_pitch_label.setText("Pitch: 0.0°")
            
            self.ros_manager.rotate_observer_camera(yaw)
            
        except Exception as e:
            print(f"Error rotando cámara: {e}")

    def reset_camera(self):
        """Resetear la cámara a posición que enfoca al dron"""
        self.ui.joystick.reset()
        self.ros_manager.reset_observer_camera()
        
    def toggle_climate_panel(self):
        if self.right_panel_expanded:
            self.ui.climate_panel.right_panel_frame.hide()
            self.right_panel_expanded = False
        else:
            self.ui.climate_panel.right_panel_frame.show()
            self.right_panel_expanded = True
            
    def show_drone_view(self):
        if self.right_panel_expanded:
            self.toggle_climate_panel()
            
    def apply_climate_settings(self):
        ambient = {color: slider.value()/100.0 for color, slider in self.ui.climate_panel.ambient_sliders.items()}
        clouds = {name: spinbox.value() for name, spinbox in self.ui.climate_panel.cloud_controls.items()}
        wind = {axis: spinbox.value() for axis, spinbox in self.ui.climate_panel.wind_controls.items()}
        turbulence = {axis: spinbox.value() for axis, spinbox in self.ui.climate_panel.turb_controls.items()}
        
        print("=== CONFIGURACIÓN CLIMÁTICA APLICADA ===")
        print(f"Luz Ambiental: R={ambient['rojo']:.1f} G={ambient['verde']:.1f} B={ambient['azul']:.1f}")
        print(f"Nubes: Velocidad={clouds['velocidad']} Tamaño={clouds['tamaño']} Humedad={clouds['humedad']}")
        print(f"Viento: X={wind['x']} Y={wind['y']} Z={wind['z']}")
        print(f"Turbulencia: X={turbulence['x']} Y={turbulence['y']} Z={turbulence['z']}")
        
        self.show_message("✅ Configuración Aplicada", "Los cambios climáticos se han aplicado correctamente")
        
    def reset_climate_settings(self):
        for slider in self.ui.climate_panel.ambient_sliders.values():
            slider.setValue(60)
            
        self.ui.climate_panel.cloud_controls['velocidad'].setValue(5)
        self.ui.climate_panel.cloud_controls['tamaño'].setValue(0.4)
        self.ui.climate_panel.cloud_controls['humedad'].setValue(0.8)
        
        for spinbox in self.ui.climate_panel.wind_controls.values():
            spinbox.setValue(0)
            
        for spinbox in self.ui.climate_panel.turb_controls.values():
            spinbox.setValue(0)
            
        self.show_message("🔄 Valores Reseteados", "Configuración climática restablecida a valores por defecto")
        
    def toggle_cameras(self):
        if not self.subscriptions_active:
            success = True
            success = success and self.ros_manager.start_camera_subscriptions()
            success = success and self.ros_manager.start_telemetry_subscriptions()
            
            if success:
                self.ui.btn_cam.setText("🎥 Desactivar Cámaras")
                self.subscriptions_active = True
                self.show_message("Cámaras Activadas", "Suscripciones ROS iniciadas correctamente")
            else:
                self.show_message("Error", "No se pudieron iniciar las suscripciones ROS")
                self.ros_manager.stop_subscriptions()
        else:
            self.ros_manager.stop_subscriptions()
            self.ui.btn_cam.setText("🎥 Activar Cámaras")
            self.clear_camera_displays()
            self.clear_telemetry()
            self.subscriptions_active = False
            
    def show_message(self, title, message):
        msg = QMessageBox()
        msg.setWindowTitle(title)
        msg.setText(message)
        msg.setIcon(QMessageBox.Information)
        msg.exec_()
        
    def update_ui(self):
        try:
            images = self.ros_manager.current_images
            
            if 'fpv' in images and images['fpv'] is not None:
                pixmap = cv2_to_pixmap(images['fpv'], self.ui.label_fp.width(), self.ui.label_fp.height())
                if pixmap: 
                    self.ui.label_fp.setPixmap(pixmap)
                    self.ui.label_fp.setText("")
            
            if 'observer' in images and images['observer'] is not None:
                pixmap = cv2_to_pixmap(images['observer'], self.ui.label_obs.width(), self.ui.label_obs.height())
                if pixmap: 
                    self.ui.label_obs.setPixmap(pixmap)
                    self.ui.label_obs.setText("")
            
            if 'tpv' in images and images['tpv'] is not None:
                pixmap = cv2_to_pixmap(images['tpv'], self.ui.label_tp.width(), self.ui.label_tp.height())
                if pixmap: 
                    self.ui.label_tp.setPixmap(pixmap)
                    self.ui.label_tp.setText("")
            
            data = self.ros_manager.telemetry_data
            if data['altitude'] is not None:
                self.ui.labels['alt'].setText(f"📏 Alt: {data['altitude']:.1f}m")
            if data['battery'] is not None:
                self.ui.labels['batt'].setText(f"🔋 Batt: {data['battery']:.0f}%")
            if data['speed'] is not None:
                self.ui.labels['speed'].setText(f"💨 Vel: {data['speed']:.1f}m/s")
            if data['mode'] is not None and data['armed'] is not None:
                armed_text = "ARMED" if data['armed'] else "DISARMED"
                self.ui.labels['mode'].setText(f"🚀 {data['mode']} | {armed_text}")
            if data['voltage'] is not None:
                self.ui.labels['voltage'].setText(f"⚡ Volt: {data['voltage']:.1f}V")
            if data['current'] is not None:
                self.ui.labels['current'].setText(f"🔌 Curr: {data['current']:.1f}A")
            if data['gps'] is not None:
                gps = data['gps']
                self.ui.labels['gps'].setText(f"📍 GPS: {gps['lat']:.4f}, {gps['lon']:.4f}")
            if data['heading'] is not None:
                self.ui.labels['heading'].setText(f"🧭 Head: {data['heading']:.0f}°")
                
        except Exception as e:
            print(f"Error actualizando UI: {e}")
            
    def clear_camera_displays(self):
        self.ui.label_fp.setText("SIN SEÑAL\nVista Primera Persona")
        self.ui.label_fp.setPixmap(None)
        self.ui.label_obs.setText("SIN SEÑAL\nVista Observer")
        self.ui.label_obs.setPixmap(None)
        self.ui.label_tp.setText("SIN SEÑAL\nVista Tercera Persona")
        self.ui.label_tp.setPixmap(None)
        
    def clear_telemetry(self):
        self.ui.labels['alt'].setText("📏 Alt: -- m")
        self.ui.labels['batt'].setText("🔋 Batt: -- %")
        self.ui.labels['speed'].setText("💨 Vel: -- m/s")
        self.ui.labels['mode'].setText("🚀 Modo: -- | Arm: --")
        self.ui.labels['voltage'].setText("⚡ Volt: -- V")
        self.ui.labels['current'].setText("🔌 Curr: -- A")
        self.ui.labels['gps'].setText("📍 GPS: --, --")
        self.ui.labels['heading'].setText("🧭 Head: --°")
        
    def closeEvent(self, event):
        self.ros_manager.stop_subscriptions()
        self.camera_timer.stop()
        self.map_timer.stop()
        event.accept()