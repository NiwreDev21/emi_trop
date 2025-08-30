# Configuración de topics ROS
CAMERA_TOPICS = {
    "observer": "/observer_camera/observer_camera/image_raw",
    "fpv": "/iris/rear_follow_cam/rear_follow_cam/image_raw",
    "tpv": "/webcam/image_raw"
}

TELEMETRY_TOPICS = {
    "pose": "/mavros/local_position/pose",
    "battery": "/mavros/battery",
    "state": "/mavros/state",
    "velocity": "/mavros/local_position/velocity_local",
    "imu": "/mavros/imu/data",
    "gps": "/mavros/global_position/global",
    "attitude": "/mavros/attitude",
    "sys_status": "/mavros/sys_status",
    "vfr_hud": "/mavros/vfr_hud",
    "extended_state": "/mavros/extended_state"
}

CONTROL_TOPICS = {
    "setpoint_position": "/mavros/setpoint_position/local",
    "setpoint_velocity": "/mavros/setpoint_velocity/cmd_vel",
    "rc_override": "/mavros/rc/override",
    "arming": "/mavros/cmd/arming",
    "mode": "/mavros/set_mode"
}

GAZEBO_SERVICE = "/gazebo/set_model_state"

# Configuración de UI
WINDOW_TITLE = "Control de Dron EMI - Estación Terrena"
WINDOW_SIZE = (1800, 1200)