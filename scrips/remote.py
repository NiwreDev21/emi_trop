import socket
import qrcode
import threading
import tkinter as tk
from PIL import Image, ImageTk

# Variables globales para mostrar datos en tiempo real
estado_gamepad = {"joystick_x": 0, "joystick_y": 0, "botones": ""}

def parsear_datos(data):
    """
    Convierte los bytes recibidos del Remote Gamepad a valores legibles.
    Esto depende del protocolo del gamepad.
    Aquí usamos un ejemplo simple: interpretamos cada byte como entero.
    """
    valores = list(data)
    estado_gamepad["joystick_x"] = valores[0] if len(valores) > 0 else 0
    estado_gamepad["joystick_y"] = valores[1] if len(valores) > 1 else 0
    estado_gamepad["botones"] = " ".join(str(v) for v in valores[2:]) if len(valores) > 2 else ""

def servidor_gamepad(host="0.0.0.0", port=35141):
    """Servidor TCP que espera conexiones del Remote Gamepad"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(1)
    print(f"🟢 Servidor escuchando en {host}:{port}")

    while True:
        conn, addr = sock.accept()
        print(f"📲 Conexión recibida desde {addr}")

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                parsear_datos(data)
        except Exception as e:
            print(f"⚠️ Error en conexión: {e}")
        finally:
            print(f"❌ Cliente {addr} desconectado")
            conn.close()

def actualizar_gui(lbl_joystick, lbl_botones):
    """Actualiza los valores mostrados en la ventana"""
    lbl_joystick.config(text=f"Joystick: X={estado_gamepad['joystick_x']}  Y={estado_gamepad['joystick_y']}")
    lbl_botones.config(text=f"Botones: {estado_gamepad['botones']}")
    # Llamar a esta función cada 100ms
    lbl_joystick.after(100, actualizar_gui, lbl_joystick, lbl_botones)

def iniciar_remote_gamepad(port=35141):
    # Obtener IP local
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip_local = s.getsockname()[0]
    except Exception:
        ip_local = "127.0.0.1"
    finally:
        s.close()

    qr_text = f"https://c.remotegamepad.com?c={ip_local}_{port}"
    print(f"📡 QR generado: {qr_text}")

    qr = qrcode.make(qr_text)

    # Ventana principal
    root = tk.Tk()
    root.title("Remote Gamepad - QR Code y Estado")

    img = ImageTk.PhotoImage(qr)
    lbl_qr = tk.Label(root, image=img)
    lbl_qr.pack(pady=10)

    lbl_joystick = tk.Label(root, text="Joystick: X=0  Y=0", font=("Arial", 14))
    lbl_joystick.pack(pady=5)

    lbl_botones = tk.Label(root, text="Botones:", font=("Arial", 14))
    lbl_botones.pack(pady=5)

    # Iniciar servidor en hilo separado
    threading.Thread(target=servidor_gamepad, args=("0.0.0.0", port), daemon=True).start()

    # Actualizar GUI en tiempo real
    actualizar_gui(lbl_joystick, lbl_botones)

    root.mainloop()

if __name__ == "__main__":
    iniciar_remote_gamepad()
