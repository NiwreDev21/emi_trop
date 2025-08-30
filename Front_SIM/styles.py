# styles.py
MAIN_STYLESHEET = """
QWidget {
    background: #1b1b1b; 
    color: #eee; 
    font-family: 'Segoe UI', Tahoma;
    font-size: 12px;
}
QLabel {
    color: #eee;
}
QGroupBox {
    font-weight: bold;
    border: 2px solid #444;
    border-radius: 8px;
    margin-top: 10px;
    padding-top: 15px;
    background: #2b2b2b;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px 0 5px;
    color: #00CCCC;
}
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #00CCCC, stop:1 #00AAAA);
    border: 1px solid #00AAAA;
    color: white;
    padding: 8px 12px;
    border-radius: 6px;
    font-weight: bold;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #66DDDD, stop:1 #00CCCC);
}
QPushButton:pressed {
    background: #44BBBB;
}
QSlider::groove:horizontal {
    border: 1px solid #444;
    height: 8px;
    background: #333;
    border-radius: 4px;
}
QSlider::handle:horizontal {
    background: #00CCCC;
    border: 1px solid #00AAAA;
    width: 18px;
    margin: -5px 0;
    border-radius: 9px;
}
QDoubleSpinBox {
    background: #333;
    border: 1px solid #444;
    color: #eee;
    padding: 5px;
    border-radius: 4px;
}
QFrame {
    background: #222;
    border-radius: 5px;
}
"""

CAMERA_LABEL_STYLE = """
QLabel {
    border: 2px solid #444;
    border-radius: 8px;
    background-color: #2b2b2b;
    font-size: 14px;
    padding: 10px;
    color: #bbb;
}
"""

TELEMETRY_BAR_STYLE = """
background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
    stop:0 #222, stop:1 #333);
border-radius: 8px;
padding: 5px;
border: 1px solid #444;
"""

TELEMETRY_LABEL_STYLE = """
font-size: 14px; 
padding: 8px; 
color: #eee; 
background: #333; 
border-radius: 5px;
border: 1px solid #444;
"""

# Estilos adicionales para elementos específicos
MENU_FRAME_STYLE = """
background-color: #222; 
border-radius: 5px;
border: 1px solid #444;
"""

CLIMATE_PANEL_STYLE = """
background-color: #222; 
border-radius: 5px;
border: 1px solid #444;
"""

BUTTON_STYLE_PRIMARY = """
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #00CCCC, stop:1 #00AAAA);
    border: 1px solid #00AAAA;
    color: white;
    padding: 10px;
    border-radius: 6px;
    font-weight: bold;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #66DDDD, stop:1 #00CCCC);
}
QPushButton:pressed {
    background: #44BBBB;
}
"""

BUTTON_STYLE_SECONDARY = """
QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #444, stop:1 #333);
    border: 1px solid #444;
    color: #eee;
    padding: 8px 12px;
    border-radius: 6px;
    font-weight: bold;
}
QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 #555, stop:1 #444);
}
QPushButton:pressed {
    background: #222;
}
"""

TITLE_LABEL_STYLE = """
font-size: 16px; 
font-weight: bold; 
color: #00CCCC; 
padding: 10px;
"""