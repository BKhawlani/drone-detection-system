🚁 Real-Time Drone Detection & Physical Alert System (ESP32 + MQTT)
📌 Project Overview

With the rapid increase in the use of drones in civilian and restricted areas, there is a growing need for low-cost, real-time drone detection systems that can operate efficiently and trigger physical alerts when a drone is detected.

This project presents a complete end-to-end drone detection system, starting from dataset collection and model training, all the way to real-time deployment, decision thresholding, and integration with a real physical alarm using ESP32 via MQTT.

The ultimate goal of this project is to contribute toward the development of affordable and scalable drone detection solutions, especially for environments with limited resources.

🔍 1. Dataset Research & Preparation
📊 Data Collection

Finding high-quality drone datasets was one of the main challenges of this project. The dataset was collected from multiple sources, including:

Public drone image datasets (open-source & research datasets)

Aerial images containing drones in different:

Altitudes

Backgrounds (urban, sky, buildings)

Lighting conditions

Drone sizes and orientations

To improve model robustness, non-drone images were also included to reduce false positives.

🏷️ Data Annotation

All images were manually or semi-automatically labeled using bounding boxes:

Class: drone

Annotation format compatible with YOLO (Ultralytics)

This ensured precise localization and accurate object detection performance.

🔄 Data Preprocessing & Augmentation

To enhance generalization, several augmentation techniques were applied:

Image resizing

Brightness and contrast variation

Horizontal flipping

Background complexity variation

🧠 2. Model Selection & Training
🤖 Model Architecture

The system uses YOLO (You Only Look Once) via the Ultralytics framework, chosen for:

High inference speed

Excellent performance on small objects

Real-time deployment capability

🏋️ Training Process

Framework: Ultralytics YOLO

Input: Annotated drone images

Output: Bounding boxes + confidence scores

Loss optimization: Localization + classification loss

The model was trained until stable convergence was achieved with satisfactory accuracy.

🎯 3. Real-Time Detection & Threshold Logic
📸 Frame-by-Frame Inference

The trained model is applied directly to live video streams (camera feed):

Capture frame

Run inference

Extract bounding boxes and confidence scores

Draw detection overlays in real-time

⚖️ Confidence Thresholding

To avoid false alarms, a confidence threshold was introduced:

If confidence ≥ threshold → Drone detected
Else → Ignore detection


This threshold was experimentally tuned to balance:

Detection accuracy

False positive reduction

🔔 4. Physical Alert System (ESP32 Integration)
🌐 Communication via MQTT

To bridge software detection with hardware action, MQTT was used:

Python application → publishes detection events

ESP32 → subscribes to alert topic

This architecture allows:

Low latency

Lightweight communication

Scalability to multiple devices

🔌 ESP32 Alarm System

When a drone is detected:

Detection event is published via MQTT

ESP32 receives the signal

A physical alarm is triggered (buzzer / LED)

Alert continues while detection persists

This transforms the system from a software-only solution into a real-world operational system.

🧩 5. System Architecture
Camera → YOLO Detection → Threshold Check
            ↓
        MQTT Publish
            ↓
          ESP32
            ↓
       Physical Alarm

🛠️ Technologies Used

Python

OpenCV

Ultralytics YOLO

NumPy

MQTT (paho-mqtt)

ESP32

Computer Vision & Real-Time Systems

🎯 Project Objectives

Real-time drone detection

Low-cost hardware integration

Physical alert triggering

Scalable architecture

Practical deployment readiness

🚀 Future Improvements

Multi-class detection (birds vs drones)

Sound-based drone detection fusion

Edge AI deployment

Multi-camera support

Battery-powered ESP32 modules

🌍 Vision

This project aims to support the development of affordable drone detection devices, making advanced surveillance technology accessible even in low-resource environments.

📌 Demo

📷 Demo GIF available in the repository showing real-time detection and physical alert activation.

👤 Author

Bashar Alkhawlani
Computer Engineering
AI & Computer Vision Enthusiast

⭐ Final Note

If you find this project useful, feel free to ⭐ the repository and contribute.