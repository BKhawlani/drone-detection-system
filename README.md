# 🚁 Real-Time Drone Detection & Physical Alert System  
### ESP32 • MQTT • YOLO • Computer Vision

---

## 📌 Project Overview
With the increasing use of drones in both civilian and restricted areas, the need for **low-cost, real-time drone detection systems** has become critical.

This project presents a **complete end-to-end drone detection system**, starting from **dataset research and model training**, all the way to **real-time detection**, **confidence thresholding**, and **integration with a real physical alarm using ESP32 via MQTT**.

The main objective is to support the development of **affordable and scalable drone detection devices**, especially for environments with limited resources.

---

## 🔍 Dataset Research & Preparation

### 📊 Data Collection
Drone images were collected from multiple public and research datasets, covering different:

- Drone sizes and models  
- Altitudes and distances  
- Backgrounds (sky, urban areas, buildings)  
- Lighting and weather conditions  

Non-drone images were also included to reduce false positives.

---

### 🏷️ Data Annotation
- All images were labeled using **bounding boxes**
- Single class used: `drone`
- Annotation format compatible with **YOLO (Ultralytics)**

---

### 🔄 Data Preprocessing & Augmentation
- Image resizing  
- Brightness & contrast variation  
- Horizontal flipping  
- Background complexity variation  

---

## 🧠 Model Training

### 🤖 Model Architecture
The system is based on **YOLO (You Only Look Once)** using the **Ultralytics framework**, selected for:

- High inference speed  
- Strong performance on small objects  
- Real-time detection capability  

---

### 🏋️ Training Process
- Framework: `Ultralytics YOLO`
- Input: Annotated drone dataset
- Output: Bounding boxes with confidence scores
- Optimization: Localization + classification loss

---

## 🎯 Real-Time Detection Logic

### 📸 Frame-by-Frame Inference
The trained model processes live camera input:

1. Capture video frame  
2. Run YOLO inference  
3. Extract bounding boxes & confidence scores  
4. Draw detection overlays in real-time  

---

### ⚖️ Confidence Threshold
To minimize false alarms, a confidence threshold is applied:

```text
if confidence >= threshold:
    Drone Detected
else:
    Ignore detection
The threshold was tuned experimentally to balance accuracy and reliability.
```
---
## 🔔 Hardware Integration (ESP32)
### 🌐 MQTT Communication
To connect the detection system with real hardware:

Python publishes detection events via MQTT

ESP32 subscribes to the alert topic

Lightweight and low-latency communication
---

### 🔌 Physical Alarm System
When a drone is detected:

Detection signal is published

ESP32 receives the message

Physical alarm (buzzer / LED) is activated

Alarm remains active while detection continues

---
## 🧩 System Architecture

Camera
  ↓
YOLO Detection
  ↓
Confidence Threshold
  ↓
MQTT Publish
  ↓
ESP32
  ↓
Physical Alarm

---

### 🛠️ Technologies Used
Python

OpenCV

Ultralytics YOLO

NumPy

MQTT (paho-mqtt)

ESP32

---

## 🚀 Future Improvements
Multi-class detection (drones vs birds)

Edge AI deployment

Multi-camera support

Sensor fusion (audio / radar)

Battery-powered ESP32 module

---


## 👤 Author
### Bashar Alkhawlani
### Computer Engineering – AI & Computer Vision

---

### ⭐ Support
If you find this project useful, please consider giving it a ⭐ on GitHub.
