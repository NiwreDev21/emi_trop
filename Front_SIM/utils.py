import cv2
import subprocess
from threading import Thread
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt
import os
import time

def cv2_to_pixmap(cv_img, width, height):
    try:
        if cv_img is None:
            return None
            
        if cv_img.size == 0:
            return None
            
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        if w == 0 or h == 0:
            return None
            
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        if qt_image.isNull():
            return None
            
        pixmap = QPixmap.fromImage(qt_image)
        
        if pixmap.isNull():
            return None
            
        return pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
    except Exception as e:
        print(f"Error converting CV image to pixmap: {e}")
        return None

def run_command(command, daemon=True):
    def run():
        try:
            process = subprocess.Popen(
                command, 
                shell=True if isinstance(command, str) else False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if daemon else None
            )
            if not daemon:
                process.wait()
        except Exception as e:
            print(f"Error ejecutando comando {command}: {e}")
    
    if daemon:
        Thread(target=run, daemon=True).start()
    else:
        run()

# Función para iniciar QGroundControl
def launch_qgroundcontrol():
    """Inicia QGroundControl desde la ubicación especificada"""
    try:
        qgc_path = "/home/niwre21/QGroundControl.AppImage"
        if os.path.exists(qgc_path):
            run_command(f"chmod +x {qgc_path} && {qgc_path}", daemon=True)
            return True
        else:
            print(f"QGroundControl no encontrado en: {qgc_path}")
            return False
    except Exception as e:
        print(f"Error iniciando QGroundControl: {e}")
        return False

# NUEVA FUNCIÓN: Iniciar ROS Master automáticamente
def start_ros_master():
    """Inicia el ROS Master si no está corriendo"""
    try:
        # Verificar si ROS Master ya está corriendo
        result = subprocess.run(['roscore', 'check'], 
                              capture_output=True, text=True, timeout=5)
        if "roscore is running" in result.stdout:
            print("ROS Master ya está ejecutándose")
            return True
    except:
        pass
    
    try:
        print("Iniciando ROS Master...")
        # Iniciar roscore en segundo plano
        process = subprocess.Popen(['roscore'],
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE,
                                 preexec_fn=os.setsid)
        
        # Esperar a que ROS Master esté listo
        timeout = 15  # segundos
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Intentar comunicarse con ROS Master
                import rosgraph
                if rosgraph.is_master_online():
                    print("ROS Master iniciado correctamente")
                    return True
                time.sleep(0.5)
            except:
                time.sleep(0.5)
        
        print("Timeout al iniciar ROS Master")
        return False
        
    except Exception as e:
        print(f"Error al iniciar ROS Master: {e}")
        return False

# Función para verificar si ROS está corriendo
def is_ros_running():
    try:
        # Método más robusto para verificar ROS
        import rosgraph
        return rosgraph.is_master_online()
    except:
        try:
            subprocess.check_output("pgrep -f roscore", shell=True)
            return True
        except subprocess.CalledProcessError:
            return False

# Comandos del sistema (tus funciones existentes)
def launch_ros():
    run_command("roslaunch emi_trop emilaunch.launch", daemon=True)

def launch_ardupilot():
    run_command("cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --location emi", daemon=True)

def launch_apm():
    run_command("roslaunch emi_trop apm.launch", daemon=True)

def close_gazebo():
    run_command("pkill -f gzserver", daemon=True)
    run_command("pkill -f gzclient", daemon=True)
    run_command("pkill -f arducopter", daemon=True)