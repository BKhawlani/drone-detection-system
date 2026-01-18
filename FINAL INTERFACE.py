import sys
import cv2
import time
import json
import numpy as np
from datetime import datetime
from collections import deque
from typing import List, Tuple, Optional
from dataclasses import dataclass

from ultralytics import YOLO
import paho.mqtt.client as mqtt

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton,
    QFileDialog, QVBoxLayout, QHBoxLayout, QWidget,
    QGroupBox, QGridLayout, QSlider, QSpinBox,
    QLineEdit, QCheckBox, QTextEdit, QSplitter,
    QFrame, QMessageBox
)
from PyQt5.QtGui import QImage, QPixmap, QFont, QIcon
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer



# CONFIGURATION & DATA CLASSES
 
@dataclass
class DroneDetection:
    id: int
    x_center: int
    y_center: int
    width: int
    height: int
    confidence: float
    timestamp: str
    bbox: Tuple[int, int, int, int]


class Config:

    # MQTT Settings
    MQTT_BROKER = "192.168.43.38"
    MQTT_PORT = 1883
    MQTT_TOPIC_COMMAND = "drone/command"
    MQTT_TOPIC_COORDINATES = "drone/coordinates"

    
    # Coordinate Publishing
    COORDINATE_PUBLISH_RATE = 0.5  # Publish every 0.5 seconds (2 Hz)
    COORDINATE_CHANGE_THRESHOLD = 20  # Only publish if moved > 20 pixels
    
    # Detection Settings
    CONFIDENCE_THRESHOLD = 0.4
    IOU_THRESHOLD = 0.5 #kareler tekrar engelleme 
    MIN_DETECTION_AREA = 100
    
    # Visualization
    COLORS = {
        'detection': (0, 255, 0),
        'no_detection': (128, 128, 128),
        'text': (255, 255, 255)
    }
    
    # Performance
    FRAME_SKIP = 1
    HISTORY_SIZE = 50


 
# MQTT MANAGER WITH COORDINATE HANDLING
 
class MQTTManager:
    
    def __init__(self, broker: str, port: int):
        self.broker = broker
        self.port = port
        self.client = None
        self.connected = False
        
        # Coordinate throttling
        self.last_coordinate_time = 0
        self.last_x = None
        self.last_y = None
        
        self._connect()
    
    def _connect(self):
        """Connect to MQTT broker"""
        try:
            self.client = mqtt.Client(client_id=f"drone_detector_{int(time.time())}")
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            print(f"Connecting to MQTT broker: {self.broker}:{self.port}")
        except Exception as e:
            print(f"MQTT connection error: {e}")
            self.connected = False
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print("✓ MQTT connected successfully")
        else:
            self.connected = False
            print(f"✗ MQTT connection failed (code: {rc})")
    
    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        print("MQTT disconnected")
    
    def publish_command(self, command: str):
        if not self.connected or not self.client:
            return False
        
        payload = {
            "command": command,
            "timestamp": datetime.now().isoformat()
        }
        
        try:
            self.client.publish(
                Config.MQTT_TOPIC_COMMAND, #topic
                json.dumps(payload), #icerik
                qos=1, # kayipolmadan gonderme
                retain=True 
            )
            print(f" Command: {command}")
            return True
        except Exception as e:
            print(f"MQTT publish error: {e}")
            return False
    
    def publish_coordinates(self, x: int, y: int, width: int, height: int, confidence: float):
        if not self.connected or not self.client:
            return False
        
        current_time = time.time()
        
        # Throttle by time
        if current_time - self.last_coordinate_time < Config.COORDINATE_PUBLISH_RATE:
            return False
        
        # Throttle by position change
        if self.last_x is not None and self.last_y is not None:
            distance = ((x - self.last_x) ** 2 + (y - self.last_y) ** 2) ** 0.5
            if distance < Config.COORDINATE_CHANGE_THRESHOLD:
                return False
        
        # Publish coordinates
        payload = {
            "x": x,
            "y": y,
            "width": width,
            "height": height,
            "confidence": round(confidence, 2),
            "timestamp": datetime.now().isoformat()
        }
        
        try:
            self.client.publish(
                Config.MQTT_TOPIC_COORDINATES,
                json.dumps(payload),
                qos=0  # gonder ve sil
            )
            
            self.last_coordinate_time = current_time
            self.last_x = x
            self.last_y = y
            
            return True
        except Exception as e:
            print(f"MQTT coordinate publish error: {e}")
            return False
    

    def disconnect(self):
        """Disconnect from MQTT broker"""
        try:
            if self.client:
                self.client.loop_stop()
                self.client.disconnect()
            print("MQTT disconnected cleanly")
        except Exception:
            pass
    
    def reconnect(self, broker: str, port: int):
        """Reconnect to a different broker"""
        self.disconnect()
        self.broker = broker
        self.port = port
        self._connect()


 
# DETECTION ENHANCER | EN IYI TESPIT EDILEN DRONE BELIRLEME 
 
class DetectionEnhancer:
 
    @staticmethod
    def non_max_suppression(detections: List, iou_threshold: float = 0.5) -> List:
        if not detections:
            return []
        
        detections = sorted(detections, key=lambda x: x.confidence, reverse=True) #tespitleri siralama
        
        keep = []
        while detections:
            best = detections.pop(0)
            keep.append(best)
            
            detections = [
                det for det in detections
                if DetectionEnhancer._iou(best.bbox, det.bbox) < iou_threshold
            ]
        
        return keep
    
    @staticmethod
    def _iou(box1: Tuple, box2: Tuple) -> float:  #tespit kutular ortusme hesaplama
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i < x1_i or y2_i < y1_i:
            return 0.0
        
        inter_area = (x2_i - x1_i) * (y2_i - y1_i)
        box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
        box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = box1_area + box2_area - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0


 
# VIDEO PROCESSING THREAD
 
class VideoThread(QThread):
    
    change_pixmap = pyqtSignal(np.ndarray)  #sinyallari hazirlama 
    detection_signal = pyqtSignal(list)
    fps_signal = pyqtSignal(int)
    status_signal = pyqtSignal(str)
    coordinate_signal = pyqtSignal(int, int, int, int)  # x, y, w, h
    
    def __init__(self, source, model, mqtt_manager: MQTTManager):
        super().__init__()
        self.source = source
        self.model = model
        self.mqtt = mqtt_manager
        self.running = True
        
        self.confidence_threshold = Config.CONFIDENCE_THRESHOLD
        self.frame_skip = Config.FRAME_SKIP
        
        self.detection_id_counter = 0
        self.detection_history = deque(maxlen=Config.HISTORY_SIZE)
        self.enhancer = DetectionEnhancer()
        
        self.frame_count = 0
        self.detection_count = 0
        self.coordinate_publish_count = 0
        self.fps = 0
        
        self.current_state = "OFF"
        self.last_state = "OFF"
    
    def run(self):
        cap = cv2.VideoCapture(self.source)
        
        if not cap.isOpened():
            self.status_signal.emit("Error: Could not open video source")
            return
        
        self.status_signal.emit("Processing started...")
        fps_counter = 0
        last_fps_time = time.time()
        
        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.status_signal.emit("Video ended")
                break
            
            self.frame_count += 1
            fps_counter += 1
            
            if self.frame_count % self.frame_skip != 0:
                continue
            
            detections = self._detect_drones(frame)
            
            # Update state
            if detections:
                self.current_state = "ON"
                
                # Publish coordinates of first (best) detection
                best_detection = detections[0]
                
                # Only publish if MQTT is available and connected
                if self.mqtt and self.mqtt.connected:
                    published = self.mqtt.publish_coordinates(
                        best_detection.x_center,
                        best_detection.y_center,
                        best_detection.width,
                        best_detection.height,
                        best_detection.confidence
                    )
                    
                    if published:
                        self.coordinate_publish_count += 1
                        self.coordinate_signal.emit(
                            best_detection.x_center,
                            best_detection.y_center,
                            best_detection.width,
                            best_detection.height
                        )
            else:
                self.current_state = "OFF"
            
            # Send state change (with MQTT check)
            if self.current_state != self.last_state:
                if self.mqtt and self.mqtt.connected:
                    self.mqtt.publish_command(self.current_state)
                self.last_state = self.current_state
            
            frame = self._draw_visualizations(frame, detections)
            
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                self.fps = fps_counter
                fps_counter = 0
                last_fps_time = current_time
                self.fps_signal.emit(self.fps)
            
            self.change_pixmap.emit(frame)
            self.detection_signal.emit(detections)
        
        cap.release()
        self.status_signal.emit("Processing stopped")
    
    def _detect_drones(self, frame: np.ndarray) -> List[DroneDetection]:
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        
        detections = []
        timestamp = datetime.now().isoformat()
        
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                
                if conf < 0.3:
                    continue
                
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                width = x2 - x1
                height = y2 - y1
                
                if width * height < Config.MIN_DETECTION_AREA:
                    continue
                
                detection = DroneDetection(
                    id=self.detection_id_counter,
                    x_center=x_center,
                    y_center=y_center,
                    width=width,
                    height=height,
                    confidence=conf,
                    timestamp=timestamp,
                    bbox=(x1, y1, x2, y2)
                )
                
                detections.append(detection)
                self.detection_id_counter += 1
                self.detection_count += 1
        
        detections = self.enhancer.non_max_suppression(detections, Config.IOU_THRESHOLD)#tespit kareler cakismayi engelleme
        self.detection_history.append(detections)
        
        return detections
    
    def _draw_visualizations(self, frame: np.ndarray, detections: List[DroneDetection]) -> np.ndarray:
        h, w = frame.shape[:2]
        
        if detections:
            for det in detections:
                x1, y1, x2, y2 = det.bbox
                color = Config.COLORS['detection']
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                label = f"Drone: {det.confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Draw center crosshair
                cv2.circle(frame, (det.x_center, det.y_center), 5, color, -1)
                cv2.line(frame, (det.x_center - 15, det.y_center), 
                        (det.x_center + 15, det.y_center), color, 2)
                cv2.line(frame, (det.x_center, det.y_center - 15), 
                        (det.x_center, det.y_center + 15), color, 2)
                
                # Draw coordinates
                coord_text = f"({det.x_center}, {det.y_center})"
                cv2.putText(frame, coord_text, (x1, y2 + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Status indicator (compact)
            cv2.rectangle(frame, (w - 120, 10), (w - 10, 45), (0, 150, 0), -1)
            cv2.putText(frame, "DETECTED", (w - 110, 32),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cv2.rectangle(frame, (w - 120, 10), (w - 10, 45), (70, 70, 70), -1)
            cv2.putText(frame, "NO DRONE", (w - 110, 32),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def update_confidence(self, value: float):
        self.confidence_threshold = value
    
    def update_frame_skip(self, value: int):
        self.frame_skip = value
    
    def stop(self):
        self.running = False
        self.quit()
        self.wait()


 
# IMPROVED UI APPLICATION
 
class DroneDetectionApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Drone Detection System")
        self.setGeometry(100, 50, 1520, 820)
        self.setStyleSheet("background-color: #1a1a2e;")
        
        self.model = None
        self.mqtt = None
        self.thread = None
        
        self.last_video_source = None
        self.is_last_source_camera = False
        
        self._load_model()
        self._setup_ui()
        
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_status)
        self.update_timer.start(1000)
    
    def _load_model(self):
        try:
            self.model = YOLO("Models/drone_detect.pt")
            print("✓ Model loaded successfully")
        except Exception as e:
            QMessageBox.critical(self, "Model Error", f"Failed to load model: {e}")
            sys.exit(1)
    
    def _setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(12, 12, 12, 12)
        
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        left_panel = self._create_control_panel()
        splitter.addWidget(left_panel)
        
        right_panel = self._create_video_panel()
        splitter.addWidget(right_panel)
        
        splitter.setSizes([380, 1140])
    
    def _create_control_panel(self) -> QWidget:
        panel = QWidget()
        panel.setMaximumWidth(400)
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)
        layout.setContentsMargins(6, 6, 6, 6)
        
        # Title - slightly larger
        title = QLabel("Drone Detection System")
        title.setFont(QFont("Arial", 15, QFont.Bold))
        title.setStyleSheet("color: #ffffff; background-color: #0f3460; padding: 10px; border-radius: 6px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        layout.addWidget(self._create_source_group())
        layout.addWidget(self._create_mqtt_connection_group())
        layout.addWidget(self._create_settings_group())
        layout.addWidget(self._create_manual_control_group())
        
        self.stats_group = self._create_stats_group()
        layout.addWidget(self.stats_group)
        
        layout.addWidget(self._create_buttons())
        
        layout.addStretch()
        
        return panel
    
    def _create_mqtt_connection_group(self) -> QGroupBox:
        group = QGroupBox("MQTT Connection")
        group.setStyleSheet(self._group_style())
        
        layout = QGridLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(10, 18, 10, 10)
        
        # Labels - slightly larger font
        ip_label = QLabel("Broker IP:")
        ip_label.setStyleSheet("color: #e0e0e0; font-size: 11px;")
        layout.addWidget(ip_label, 0, 0)
        
        self.mqtt_ip_input = QLineEdit(Config.MQTT_BROKER)
        self.mqtt_ip_input.setStyleSheet(self._input_style())
        self.mqtt_ip_input.setFixedHeight(28)
        layout.addWidget(self.mqtt_ip_input, 0, 1)
        
        port_label = QLabel("Port:")
        port_label.setStyleSheet("color: #e0e0e0; font-size: 11px;")
        layout.addWidget(port_label, 1, 0)
        
        self.mqtt_port_input = QSpinBox()
        self.mqtt_port_input.setMinimum(1)
        self.mqtt_port_input.setMaximum(65535)
        self.mqtt_port_input.setValue(Config.MQTT_PORT)
        self.mqtt_port_input.setStyleSheet(self._spinbox_style())
        self.mqtt_port_input.setFixedHeight(28)
        layout.addWidget(self.mqtt_port_input, 1, 1)
        
        self.btn_mqtt_connect = QPushButton(" Connect")
        self.btn_mqtt_connect.setStyleSheet(self._button_style("#17a2b8"))
        self.btn_mqtt_connect.setFixedHeight(32)
        self.btn_mqtt_connect.clicked.connect(self._connect_mqtt)
        layout.addWidget(self.btn_mqtt_connect, 2, 0, 1, 2)
        
        self.mqtt_status_label = QLabel("Status: Not Connected")
        self.mqtt_status_label.setStyleSheet("color: #ff6b6b; padding: 4px; font-size: 11px; font-weight: bold;")
        layout.addWidget(self.mqtt_status_label, 3, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def _create_source_group(self) -> QGroupBox:
        group = QGroupBox("Video Source")
        group.setStyleSheet(self._group_style())
        
        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(10, 18, 10, 10)
        
        self.btn_camera = QPushButton("Camera")
        self.btn_camera.setStyleSheet(self._button_style("#28a745"))
        self.btn_camera.setFixedHeight(32)
        self.btn_camera.clicked.connect(self.open_camera)
        layout.addWidget(self.btn_camera)
        
        self.btn_video = QPushButton(" Video File")
        self.btn_video.setStyleSheet(self._button_style("#007bff"))
        self.btn_video.setFixedHeight(32)
        self.btn_video.clicked.connect(self.open_video)
        layout.addWidget(self.btn_video)
        
        self.source_label = QLabel("No source selected")
        self.source_label.setStyleSheet("color: #8b949e; padding: 4px; font-size: 10px;")
        layout.addWidget(self.source_label)
        
        group.setLayout(layout)
        return group
    
    def _create_settings_group(self) -> QGroupBox:
        group = QGroupBox("Detection Settings")
        group.setStyleSheet(self._group_style())
        
        layout = QGridLayout()
        layout.setSpacing(8)
        layout.setContentsMargins(10, 18, 10, 10)
        
        conf_label = QLabel("Confidence:")
        conf_label.setStyleSheet("color: #e0e0e0; font-size: 11px;")
        layout.addWidget(conf_label, 0, 0)
        
        self.conf_slider = QSlider(Qt.Horizontal)
        self.conf_slider.setMinimum(10)
        self.conf_slider.setMaximum(95)
        self.conf_slider.setValue(40)
        self.conf_slider.setStyleSheet(self._slider_style())
        self.conf_slider.setFixedHeight(24)
        self.conf_slider.valueChanged.connect(self._on_conf_changed)
        layout.addWidget(self.conf_slider, 0, 1)
        
        self.conf_value = QLabel("0.40")
        self.conf_value.setStyleSheet("color: #00ff88; font-weight: bold; font-size: 11px;")
        layout.addWidget(self.conf_value, 0, 2)
        
        skip_label = QLabel("Frame Skip:")
        skip_label.setStyleSheet("color: #e0e0e0; font-size: 11px;")
        layout.addWidget(skip_label, 1, 0)
        
        self.skip_spin = QSpinBox()
        self.skip_spin.setMinimum(1)
        self.skip_spin.setMaximum(10)
        self.skip_spin.setValue(1)
        self.skip_spin.setStyleSheet(self._spinbox_style())
        self.skip_spin.setFixedHeight(28)
        self.skip_spin.valueChanged.connect(self._on_skip_changed)
        layout.addWidget(self.skip_spin, 1, 1, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def _create_manual_control_group(self) -> QGroupBox:
        group = QGroupBox("Manual Control")
        group.setStyleSheet(self._group_style())
        
        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(10, 18, 10, 10)
        
        info_label = QLabel("Send manual commands:")
        info_label.setStyleSheet("color: #e0e0e0; padding: 4px; font-size: 10px;")
        layout.addWidget(info_label)
        
        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)
    
        self.btn_force_on = QPushButton(" ON")
        self.btn_force_on.setStyleSheet(self._button_style("#28a745"))
        self.btn_force_on.setFixedHeight(36)
        self.btn_force_on.clicked.connect(lambda: self._send_manual_command("ON"))
        button_layout.addWidget(self.btn_force_on)
        
        self.btn_force_off = QPushButton("OFF")
        self.btn_force_off.setStyleSheet(self._button_style("#dc3545"))
        self.btn_force_off.setFixedHeight(36)
        self.btn_force_off.clicked.connect(lambda: self._send_manual_command("OFF"))
        button_layout.addWidget(self.btn_force_off)
        
        layout.addLayout(button_layout)
        
        group.setLayout(layout)
        return group
    
    def _create_stats_group(self) -> QGroupBox:
        group = QGroupBox(" Statistics")
        group.setStyleSheet(self._group_style())
        
        layout = QVBoxLayout()
        layout.setSpacing(4)
        layout.setContentsMargins(10, 18, 10, 10)
        
        self.lbl_fps = self._stat_label("FPS: 0")
        self.lbl_state = self._stat_label("State: OFF")
        self.lbl_detections = self._stat_label("Total: 0")
        self.lbl_coordinates = self._stat_label("Coords Published: 0")
        self.lbl_current_pos = self._stat_label("Position: ---")
        
        for lbl in [self.lbl_fps, self.lbl_state, self.lbl_detections, 
                   self.lbl_coordinates, self.lbl_current_pos]:
            layout.addWidget(lbl)
        
        group.setLayout(layout)
        return group
    
    def _create_buttons(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(6)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(self._button_style("#dc3545"))
        self.btn_stop.setFixedHeight(38)
        self.btn_stop.clicked.connect(self.stop_video)
        layout.addWidget(self.btn_stop)
        
        self.btn_replay = QPushButton("REPLAY")
        self.btn_replay.setEnabled(False)
        self.btn_replay.setStyleSheet(self._button_style("#ff9800"))
        self.btn_replay.setFixedHeight(38)
        self.btn_replay.clicked.connect(self.replay_video)
        layout.addWidget(self.btn_replay)
        
        btn_reset = QPushButton("Reset Stats")
        btn_reset.setStyleSheet(self._button_style("#6c757d"))
        btn_reset.setFixedHeight(34)
        btn_reset.clicked.connect(self._reset_stats)
        layout.addWidget(btn_reset)
        
        return widget
    
    def _create_video_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)
        
        self.video_label = QLabel("Select video source to begin...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(750, 520)
        self.video_label.setStyleSheet("""
            QLabel {
                background-color: #16213e;
                color: #0f3460;
                font-size: 17px;
                font-weight: bold;
                border: 3px solid #0f3460;
                border-radius: 10px;
            }
        """)
        layout.addWidget(self.video_label, 3)
        
        log_group = QGroupBox("Event Log")
        log_group.setStyleSheet(self._group_style())
        log_group.setMaximumHeight(165)
        
        log_layout = QVBoxLayout()
        log_layout.setContentsMargins(6, 14, 6, 6)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #16213e;
                color: #00ff88;
                font-family: 'Courier New';
                font-size: 10px;
                border: 2px solid #0f3460;
                border-radius: 5px;
                padding: 4px;
            }
        """)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group, 1)
        
        return panel
    
    # IMPROVED STYLING METHODS
    def _group_style(self) -> str:
        return """
            QGroupBox {
                color: #00ff88;
                font-size: 12px;
                font-weight: bold;
                border: 2px solid #0f3460;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 14px;
                background-color: #16213e;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """
    
    def _button_style(self, color: str) -> str:
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                padding: 8px;
                border-radius: 6px;
                font-size: 12px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {color}dd;
            }}
            QPushButton:pressed {{
                background-color: {color}aa;
            }}
            QPushButton:disabled {{
                background-color: #4a4a4a;
                color: #8b949e;
            }}
        """
    
    def _slider_style(self) -> str:
        return """
            QSlider::groove:horizontal {
                border: 1px solid #0f3460;
                height: 8px;
                background: #16213e;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #00ff88;
                border: 2px solid #00ff88;
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }
        """
    
    def _spinbox_style(self) -> str:
        return """
            QSpinBox {
                background-color: #16213e;
                color: white;
                padding: 5px;
                border: 2px solid #0f3460;
                border-radius: 5px;
                font-size: 11px;
            }
            QSpinBox:hover {
                border-color: #00ff88;
            }
        """
    
    def _input_style(self) -> str:
        return """
            QLineEdit {
                background-color: #16213e;
                color: white;
                padding: 5px;
                border: 2px solid #0f3460;
                border-radius: 5px;
                font-size: 11px;
            }
            QLineEdit:hover {
                border-color: #00ff88;
            }
        """
    
    def _stat_label(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet("color: #e0e0e0; padding: 3px; font-size: 11px;")
        return lbl
    
      
    # MQTT METHODS
      
    def _connect_mqtt(self):
        broker = self.mqtt_ip_input.text().strip()
        port = self.mqtt_port_input.value()
        
        if not broker:
            QMessageBox.warning(self, "Invalid Input", "Please enter a broker IP address")
            return
        
        try:
            if self.mqtt:
                self.mqtt.disconnect()
            
            self.mqtt = MQTTManager(broker, port)
            self.log_message(f"Connecting to MQTT: {broker}:{port}")
            
            QTimer.singleShot(1500, self._check_mqtt_connection)
            
        except Exception as e:
            self.log_message(f" MQTT connection failed: {e}")
            self.mqtt_status_label.setText("Status: Failed")
            self.mqtt_status_label.setStyleSheet("color: #ff6b6b; padding: 4px; font-size: 11px; font-weight: bold;")
    
    def _check_mqtt_connection(self):
        if self.mqtt and self.mqtt.connected:
            self.mqtt_status_label.setText("Status: ✓ Connected")
            self.mqtt_status_label.setStyleSheet("color: #00ff88; padding: 4px; font-size: 11px; font-weight: bold;")
            self.log_message(" MQTT connected successfully")
        else:
            self.mqtt_status_label.setText("Status: ✗ Failed")
            self.mqtt_status_label.setStyleSheet("color: #ff6b6b; padding: 4px; font-size: 11px; font-weight: bold;")
            self.log_message(" MQTT connection failed")
    
    def _send_manual_command(self, command: str):
        if not self.mqtt or not self.mqtt.connected:
            QMessageBox.warning(self, "Not Connected", "Please connect to MQTT first")
            return
        
        self.mqtt.publish_command(command)
        self.log_message(f" Manual: {command}")
    
      
    # VIDEO CONTROL METHODS
      
    def open_camera(self):
        self.source_label.setText("Source: Webcam")
        self.start_video(0)
    
    def open_video(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select Video File", "",
            "Video Files (*.mp4 *.avi *.mov *.mkv *.flv);;All Files (*)"
        )
        if file_path:
            filename = file_path.split('/')[-1]
            if len(filename) > 30:
                filename = filename[:27] + "..."
            self.source_label.setText(f"Source: {filename}")
            self.start_video(file_path)
    
    def start_video(self, source):
        self.stop_video()
        
        # Auto-connect to MQTT if not connected
        if not self.mqtt:
            broker = self.mqtt_ip_input.text().strip()
            port = self.mqtt_port_input.value()
            
            if broker:
                try:
                    self.mqtt = MQTTManager(broker, port)
                    self.log_message(f" Auto-connecting to MQTT: {broker}:{port}")
                    QTimer.singleShot(1000, self._check_mqtt_connection)
                except Exception as e:
                    self.log_message(f"MQTT unavailable: {e}")
                    self.log_message("Detection will run without MQTT")
        
        self.last_video_source = source
        self.is_last_source_camera = (source == 0)
        
        self.thread = VideoThread(source, self.model, self.mqtt)
        self.thread.change_pixmap.connect(self.update_image)
        self.thread.detection_signal.connect(self.update_detections)
        self.thread.fps_signal.connect(self.update_fps)
        self.thread.status_signal.connect(self.log_message)
        self.thread.coordinate_signal.connect(self.update_coordinates)
        self.thread.finished.connect(self._on_video_finished)
        
        self.thread.update_confidence(self.conf_slider.value() / 100.0)
        self.thread.update_frame_skip(self.skip_spin.value())
        
        self.thread.start()
        
        self.btn_camera.setEnabled(False)
        self.btn_video.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_replay.setEnabled(False)
        
        self.log_message(" Detection started")
    
    def stop_video(self):
        if self.thread:
            self.thread.stop()
            self.thread = None
        
        self.btn_camera.setEnabled(True)
        self.btn_video.setEnabled(True)
        self.btn_stop.setEnabled(False)
        
        self.log_message("Stopped")
    
    def replay_video(self):
        if self.last_video_source is not None:
            if self.is_last_source_camera:
                self.source_label.setText("Source: Webcam (Replay)")
            else:
                filename = str(self.last_video_source).split('/')[-1]
                if len(filename) > 30:
                    filename = filename[:27] + "..."
                self.source_label.setText(f"Source: {filename} (Replay)")
            
            self.start_video(self.last_video_source)
            self.log_message(" Replaying")
    
    def _on_video_finished(self):
        if not self.is_last_source_camera and self.last_video_source is not None:
            self.btn_replay.setEnabled(True)
            self.log_message("Video finished")
    
      
    # UPDATE METHODS
      
    def update_image(self, frame: np.ndarray):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        
        self.video_label.setPixmap(
            pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        )
    
    def update_detections(self, detections: List[DroneDetection]):
        if self.thread:
            self.lbl_detections.setText(f"Total: {self.thread.detection_count}")
            self.lbl_state.setText(f"State: {self.thread.current_state}")
            self.lbl_coordinates.setText(f"Coords: {self.thread.coordinate_publish_count}")
    
    def update_coordinates(self, x: int, y: int, w: int, h: int):
        self.lbl_current_pos.setText(f"Position: ({x}, {y})")
    
    def update_fps(self, fps: int):
        self.lbl_fps.setText(f"FPS: {fps}")
    
    def log_message(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def _update_status(self):
        if self.mqtt and self.mqtt.connected:
            self.mqtt_status_label.setText("Status: ✓ Connected")
            self.mqtt_status_label.setStyleSheet("color: #00ff88; padding: 4px; font-size: 11px; font-weight: bold;")
    
      
      
    def _on_conf_changed(self, value):
        conf = value / 100.0
        self.conf_value.setText(f"{conf:.2f}")
        if self.thread:
            self.thread.update_confidence(conf)
    
    def _on_skip_changed(self, value):
        if self.thread:
            self.thread.update_frame_skip(value)
    
    def _reset_stats(self):
        if self.thread:
            self.thread.detection_count = 0
            self.thread.coordinate_publish_count = 0
        self.log_text.clear()
        self.log_message("📊 Stats reset")
    
      
    # CLEANUP
      
    def closeEvent(self, event):
        self.stop_video()
        if self.mqtt:
            self.mqtt.disconnect()
        event.accept()


 
# MAIN ENTRY POINT
 
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = DroneDetectionApp()
    window.show()
    
    sys.exit(app.exec_())