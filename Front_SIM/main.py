#main.pyS
#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import QApplication
from drone_gui import DroneGUI
from utils import start_ros_master, is_ros_running
import time

def main():
    try:
        # Intentar iniciar ROS Master automáticamente
        if not is_ros_running():
            print("ROS Master no detectado, iniciando automáticamente...")
            if not start_ros_master():
                print("No se pudo iniciar ROS Master. Verifica tu instalación de ROS.")
                # Puedes continuar o salir, dependiendo de qué quieras hacer
                # return 1  # Descomenta si quieres salir en caso de error
        
        # Esperar un poco para que ROS Master esté completamente listo
        time.sleep(2)
        
        # Inicializar ROS node
        if not rospy.core.is_initialized():
            rospy.init_node('drone_control_station', anonymous=True, disable_signals=True)
        
        # Verificar si ROS está realmente disponible
        try:
            rospy.get_time()  # Intenta acceder a funcionalidad de ROS
            print("ROS inicializado correctamente")
        except:
            print("Advertencia: ROS no está completamente inicializado")
        
        app = QApplication(sys.argv)
        gui = DroneGUI()
        gui.show()
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"Error al iniciar la aplicación: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()