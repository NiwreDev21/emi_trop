# SIMULADOR PARA EL ENTRENAMIENTO EN EL PILOTAJE DE UN CUADRICOPTERO EN ENTRONOS VIRTUALES
## Proyecto de Trabajo de Grado
![emi](config/INTRO.png)
## Interfaz Estacion de Control
Simulacion iniciada
![emi](config/inicio.jpeg)
![emi](config/interfaz.jpeg)
![emi](config/funcional.jpeg)

## Software Development Tutorials
[Installing Ardupilot and MAVProxy](Install/ArduPillot-20.04.md)

## Modelo 3D EMI
![emi](config/3D.png)

## Ejecucion del Simulador
Inicia el mundo de Gazebo.

```bash roslaunch emi_trop emilaunch.launch```

Ejecutar en otra terminal el SITL.

```bash cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --location emi```

## References 
http://ardupilot.org/copter/index.html

http://ardupilot.org/copter/docs/parameters.html#wpnav-parameters

http://qgroundcontrol.com/

https://discuss.ardupilot.org/

http://ardupilot.org/dev/

https://www.ros.org/
