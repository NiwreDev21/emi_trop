# Instalación de ArduPilot y MAVProxy en Ubuntu 20.04

## 🎥 Tutorial en video
[Ver en YouTube](https://youtu.be/1FpJvUVPxL0)

---

## 1️⃣ Clonar ArduPilot

En tu directorio home, ejecuta:

```bash
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
2️⃣ Instalar dependencias
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
Recarga el perfil de usuario:
. ~/.profile
3️⃣ En caso de error con git submodule update

Si el siguiente paso falla, ejecuta:
git config --global url.https://.insteadOf git://
4️⃣ Descargar la última versión de Copter
git checkout Copter-4.0.4
git submodule update --init --recursive
5️⃣ Ejecutar SITL (Software In The Loop) por primera vez

Esto configurará los parámetros iniciales:
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w