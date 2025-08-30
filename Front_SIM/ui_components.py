# ui_components.py
from PyQt5.QtWidgets import (
    QLabel, QPushButton, QFrame, QSlider, QDoubleSpinBox, 
    QHBoxLayout, QVBoxLayout, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap

def create_camera_label(title, min_width, min_height):
    """Crea un QLabel para mostrar cámaras"""
    label = QLabel(f"SIN SEÑAL\n{title}")
    label.setAlignment(Qt.AlignCenter)
    label.setFrameShape(QLabel.Box)
    label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    label.setMinimumSize(min_width, min_height)
    
    from styles import CAMERA_LABEL_STYLE
    label.setStyleSheet(CAMERA_LABEL_STYLE)
    
    return label

def create_control_group(title, layout):
    """Crea un grupo de controles con título"""
    group = QGroupBox(title)
    group.setLayout(layout)
    return group

def create_slider_control(label_text, min_val, max_val, default_val, value_changed_callback=None):
    """Crea un control deslizante con etiqueta"""
    layout = QHBoxLayout()
    lbl = QLabel(f"{label_text}:")
    slider = QSlider(Qt.Horizontal)
    slider.setRange(min_val, max_val)
    slider.setValue(default_val)
    value_label = QLabel(f"{default_val/100:.1f}")
    
    if value_changed_callback:
        slider.valueChanged.connect(value_changed_callback)
    else:
        slider.valueChanged.connect(lambda val, lbl=value_label: lbl.setText(f"{val/100:.1f}"))
    
    layout.addWidget(lbl)
    layout.addWidget(slider)
    layout.addWidget(value_label)
    
    return layout, slider, value_label

def create_spinbox_control(label_text, min_val, max_val, default_val, step=0.1, decimals=2):
    """Crea un control de spinbox con etiqueta"""
    layout = QHBoxLayout()
    lbl = QLabel(f"{label_text}:")
    spinbox = QDoubleSpinBox()
    spinbox.setRange(min_val, max_val)
    spinbox.setValue(default_val)
    spinbox.setSingleStep(step)
    spinbox.setDecimals(decimals)
    
    layout.addWidget(lbl)
    layout.addWidget(spinbox)
    
    return layout, spinbox