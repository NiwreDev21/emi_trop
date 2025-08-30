from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap
import sys

class DroneControllerUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Controller")
        self.setGeometry(100, 100, 800, 600)
        self.initUI()
    
    def initUI(self):
        layout = QVBoxLayout()
        
        try:
            self.bg_label = QLabel(self)
            pixmap = QPixmap("/mnt/data/imagen.png")
            self.bg_label.setPixmap(pixmap)
            self.bg_label.setScaledContents(True)
            layout.addWidget(self.bg_label)
        except Exception as e:
            print("Error al cargar la imagen:", e)
        
        self.setLayout(layout)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DroneControllerUI()
    window.show()
    sys.exit(app.exec_())
