# ros_manager.py
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, BatteryState, Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Point, Quaternion
from mavros_msgs.msg import State, ExtendedState, VFR_HUD, SysStatus
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_msgs.msg import Header
import tf.transformations as tft
import math
import time
import numpy as np
from utils import is_ros_running

class ROSManager:
    def __init__(self):
        self.bridge = CvBridge()
        self.subscribers = {}
        self.current_images = {}
        self.telemetry_data = {
            'altitude': 0.0,
            'battery': 0.0,
            'speed': 0.0,
            'mode': "DESCONOCIDO",
            'armed': False,
            'velocity': Vector3(0, 0, 0),
            'attitude': Vector3(0, 0, 0),
            'gps': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'heading': 0.0,
            'throttle': 0.0,
            'flight_mode': "UNKNOWN",
            'system_status': "INIT",
            'voltage': 0.0,
            'current': 0.0,
            'position': Point(0, 0, 0)
        }
        self.is_initialized = False
        self.last_update_time = time.time()
        self.last_position = None
        self.manual_override = False
        self.current_yaw = 0.0
        self.camera_update_interval = 0.008  # ≈60 Hz (16ms)
        self.last_camera_update = time.time()
        
    def init_ros(self):
        """Intenta inicializar ROS, esperando si es necesario"""
        max_attempts = 10
        attempt = 0
        
        while attempt < max_attempts:
            try:
                if not rospy.core.is_initialized():
                    rospy.init_node('ros_manager', anonymous=True, disable_signals=True)
                
                rospy.get_time()
                self.is_initialized = True
                print("ROS Manager inicializado correctamente")
                return True
                
            except Exception as e:
                attempt += 1
                print(f"Intento {attempt}/{max_attempts}: ROS no disponible - {e}")
                time.sleep(1)
        
        print("No se pudo inicializar ROS Manager después de varios intentos")
        return False
    
    def start_camera_subscriptions(self):
        """Iniciar suscripciones a las cámaras"""
        if not self.is_initialized:
            print("ROS no inicializado")
            return False
            
        from config import CAMERA_TOPICS
        
        try:
            for cam_type, topic in CAMERA_TOPICS.items():
                print(f"Suscribiéndose a: {topic}")
                self.subscribers[topic] = rospy.Subscriber(
                    topic, Image, lambda msg, ct=cam_type: self.image_callback(msg, ct)
                )
            return True
        except Exception as e:
            print(f"Error suscribiendo a cámaras: {e}")
            return False
        
    def start_telemetry_subscriptions(self):
        if not self.is_initialized:
            print("ROS no inicializado")
            return False
            
        from config import TELEMETRY_TOPICS
        
        try:
            print("Iniciando suscripciones de telemetría avanzada...")
            
            self.subscribers[TELEMETRY_TOPICS['pose']] = rospy.Subscriber(
                TELEMETRY_TOPICS['pose'], PoseStamped, self.pose_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['battery']] = rospy.Subscriber(
                TELEMETRY_TOPICS['battery'], BatteryState, self.battery_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['state']] = rospy.Subscriber(
                TELEMETRY_TOPICS['state'], State, self.state_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['velocity']] = rospy.Subscriber(
                TELEMETRY_TOPICS['velocity'], TwistStamped, self.velocity_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['imu']] = rospy.Subscriber(
                TELEMETRY_TOPICS['imu'], Imu, self.imu_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['gps']] = rospy.Subscriber(
                TELEMETRY_TOPICS['gps'], NavSatFix, self.gps_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['vfr_hud']] = rospy.Subscriber(
                TELEMETRY_TOPICS['vfr_hud'], VFR_HUD, self.vfr_hud_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['extended_state']] = rospy.Subscriber(
                TELEMETRY_TOPICS['extended_state'], ExtendedState, self.extended_state_callback
            )
            
            self.subscribers[TELEMETRY_TOPICS['sys_status']] = rospy.Subscriber(
                TELEMETRY_TOPICS['sys_status'], SysStatus, self.sys_status_callback
            )
            
            print("Suscripciones de telemetría avanzada iniciadas correctamente")
            return True
        except Exception as e:
            print(f"Error suscribiendo a telemetría avanzada: {e}")
            return False
    
    def velocity_callback(self, msg):
        try:
            self.telemetry_data['velocity'] = msg.twist.linear
        except Exception as e:
            print(f"Error en callback de velocidad: {e}")
    
    def imu_callback(self, msg):
        try:
            orientation = msg.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            euler = tft.euler_from_quaternion(quaternion)
            self.telemetry_data['attitude'] = Vector3(*euler)
        except Exception as e:
            print(f"Error en callback de IMU: {e}")
    
    def gps_callback(self, msg):
        try:
            self.telemetry_data['gps'] = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            }
        except Exception as e:
            print(f"Error en callback de GPS: {e}")
    
    def vfr_hud_callback(self, msg):
        try:
            self.telemetry_data['heading'] = msg.heading
            self.telemetry_data['throttle'] = msg.throttle
            self.telemetry_data['speed'] = msg.airspeed
        except Exception as e:
            print(f"Error en callback de VFR_HUD: {e}")
    
    def extended_state_callback(self, msg):
        try:
            self.telemetry_data['flight_mode'] = "VTOL" if msg.vtol else "MC"
        except Exception as e:
            print(f"Error en callback de estado extendido: {e}")
    
    def sys_status_callback(self, msg):
        try:
            self.telemetry_data['voltage'] = msg.voltage_battery / 1000.0
            self.telemetry_data['current'] = msg.current_battery / 100.0
        except Exception as e:
            print(f"Error en callback de estado del sistema: {e}")
    
    def stop_subscriptions(self):
        try:
            for topic, sub in list(self.subscribers.items()):
                sub.unregister()
                print(f"Desuscrito de: {topic}")
            self.subscribers.clear()
            self.current_images.clear()
        except Exception as e:
            print(f"Error desuscribiendo: {e}")
    
    def image_callback(self, msg, camera_type):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_images[camera_type] = cv_image
        except Exception as e:
            print(f"Error procesando imagen {camera_type}: {e}")
    
    def pose_callback(self, msg):
        try:
            self.telemetry_data['position'] = msg.pose.position
            self.telemetry_data['altitude'] = msg.pose.position.z
            
            current_time = time.time()
            time_diff = current_time - self.last_update_time
            
            if time_diff > 0.1:
                if self.last_position is not None:
                    dx = msg.pose.position.x - self.last_position.x
                    dy = msg.pose.position.y - self.last_position.y
                    dz = msg.pose.position.z - self.last_position.z
                    distance = math.sqrt(dx**2 + dy**2 + dz**2)
                    self.telemetry_data['speed'] = distance / time_diff
                
                self.last_position = msg.pose.position
                self.last_update_time = current_time
                
        except Exception as e:
            print(f"Error en callback de pose: {e}")
    
    def battery_callback(self, msg):
        try:
            self.telemetry_data['battery'] = msg.percentage * 100
        except Exception as e:
            print(f"Error en callback de batería: {e}")
    
    def state_callback(self, msg):
        try:
            self.telemetry_data['mode'] = msg.mode
            self.telemetry_data['armed'] = "SÍ" if msg.armed else "NO"
        except Exception as e:
            print(f"Error en callback de estado: {e}")
    
    def calculate_camera_yaw(self, drone_position):
        """Calcular el yaw CORRECTO para enfocar al dron horizontalmente"""
        try:
            # Posición fija de la cámara observer (de tu mundo SDF)
            camera_position = Point(0.67, -0.98, 0.96)
            
            # Vector desde la cámara al dron (CORREGIDO)
            dx = drone_position.x - camera_position.x
            dy = drone_position.y - camera_position.y
            
            # Calcular yaw (ángulo horizontal) - CORREGIDO
            # En Gazebo, el ángulo se calcula desde el eje X positivo (este)
            # Queremos que la cámara mire hacia el dron
            yaw = math.atan2(dy, dx)
            
            # Ajustar para que 0° sea frente a la cámara (eje Y negativo en Gazebo)
            # y girar en la dirección correcta
            yaw_corrected = yaw - math.pi/2  # Rotar 90 grados
            
            # Normalizar entre -π y π
            yaw_corrected = (yaw_corrected + math.pi) % (2 * math.pi) - math.pi
            
            return yaw_corrected
            
        except Exception as e:
            print(f"Error calculando yaw de cámara: {e}")
            return 0
    
    def rotate_observer_camera(self, yaw_degrees):
        """Rotar cámara observer solo horizontalmente"""
        try:
            if not self.manual_override:
                # Modo automático: seguir al dron
                drone_pos = self.telemetry_data['position']
                yaw_auto = self.calculate_camera_yaw(drone_pos)
                yaw_final = math.degrees(yaw_auto)
            else:
                # Modo manual: usar el joystick
                yaw_final = yaw_degrees
            
            self.current_yaw = yaw_final
            
            rospy.wait_for_service('/gazebo/set_model_state', timeout=1.0)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            req = SetModelStateRequest()
            req.model_state.model_name = "ground_observer_camera"
            
            # Posición fija de la cámara (de tu mundo SDF)
            req.model_state.pose.position.x = 0.67
            req.model_state.pose.position.y = -0.98
            req.model_state.pose.position.z = 0.96

            # Solo rotación horizontal (yaw) - CORREGIDO
            # Usar ángulo corregido para que mire hacia el dron
            q = tft.quaternion_from_euler(0, 0, math.radians(yaw_final))
            
            req.model_state.pose.orientation.x = q[0]
            req.model_state.pose.orientation.y = q[1]
            req.model_state.pose.orientation.z = q[2]
            req.model_state.pose.orientation.w = q[3]
            req.model_state.reference_frame = "world"

            response = set_state(req)
            if not response.success:
                print(f"Error rotando cámara: {response.status_message}")
                
            return yaw_final

        except rospy.ServiceException as e:
            print(f"Servicio no disponible: {e}")
        except rospy.ROSException as e:
            print(f"Timeout esperando servicio: {e}")
        except Exception as e:
            print(f"Error rotando cámara: {e}")
        return yaw_degrees

    def set_manual_override(self, manual):
        """Activar/desactivar control manual"""
        self.manual_override = manual
        
    def reset_observer_camera(self):
        """Resetear la cámara para que enfoque al dron"""
        try:
            self.manual_override = False
            drone_pos = self.telemetry_data['position']
            yaw_auto = self.calculate_camera_yaw(drone_pos)
            self.rotate_observer_camera(math.degrees(yaw_auto))
            
        except Exception as e:
            print(f"Error reseteando cámara: {e}")