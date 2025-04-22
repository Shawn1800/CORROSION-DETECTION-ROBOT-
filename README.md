# Autonomous Corrosion Detection Robot

## Project Overview
This project is an autonomous wheeled robot designed for real-time corrosion detection in industrial environments. The system integrates computer vision capabilities with a mobile robotic platform to automatically navigate facilities, identify corroded areas, and report findings through a user-friendly web interface. By combining Python, computer vision technologies, and robotics hardware, this solution provides an efficient method for monitoring infrastructure integrity.

[Photo/video of robot in action will be added later]

## Problem Statement & Solution

### Why Corrosion Detection Matters
Corrosion presents a significant challenge in industrial environments, causing structural weakening, equipment failure, and potential safety hazards. Traditional inspection methods are labor-intensive, time-consuming, and often miss early-stage corrosion in hard-to-reach areas. Early detection is crucial for preventing catastrophic failures and reducing maintenance costs in facilities like manufacturing plants, oil refineries, and infrastructure systems.

### How This Solution Improves on Manual Inspection
This autonomous robot solution transforms corrosion monitoring by:
- Enabling continuous, consistent inspection without human fatigue
- Accessing confined or hazardous spaces unsafe for human inspectors
- Providing objective, data-driven detection through machine learning algorithms
- Creating digital records of all inspections with precise location data
- Reducing inspection costs while increasing coverage and reliability
- Identifying early-stage corrosion before it becomes visible to the human eye

## System Architecture

[System diagram showing component interactions will be added later]

### Main Subsystems

#### Vision System
The vision subsystem utilizes a Raspberry Pi Camera V2 to capture video feeds of industrial surfaces. These feeds are processed through a custom computer vision pipeline built on Python and OpenCV, with YOLOv11 model inference via the Roboflow API. The system identifies different types of corrosion and provides bounding box visualization around detected areas.

#### Navigation System
The navigation system combines ultrasonic sensors for obstacle detection with motor drivers for wheel control. An Arduino Uno processes sensor data and controls the robot's movement, enabling autonomous navigation through industrial environments while avoiding obstacles. The system ensures complete coverage of inspection areas while maintaining safe operation.

#### Web Interface
A Flask-based web application provides remote monitoring and control capabilities. Users can view the live camera feed with real-time corrosion detection overlays, access sensor data, control the robot manually when needed, and review historical inspection data. The interface is designed to be intuitive and accessible from any device with a web browser.

## Technical Implementation

### Vision System (YOLOv11, Roboflow)
The corrosion detection pipeline leverages YOLOv11, a state-of-the-art object detection model, trained on a custom dataset of corrosion examples. The Roboflow Inference API handles model execution, while custom Python code processes the results to:
- Analyze video frames in real-time from the Raspberry Pi Camera
- Identify multiple types of corrosion (surface, pitting, crevice, etc.)
- Draw bounding boxes around detected corrosion areas
- Calculate confidence scores for each detection
- Track corrosion progression over time

The system is optimized for edge deployment on the Raspberry Pi 5, with tuned parameters to balance accuracy and processing speed.

### Arduino Control System
The Arduino Uno serves as the robot's motion control hub, running custom firmware that:
- Interfaces with multiple ultrasonic sensors for 360° obstacle detection
- Controls dual H-bridge motor drivers for differential steering
- Implements PID control for smooth navigation and accurate positioning
- Communicates with the Raspberry Pi via serial connection
- Manages power distribution to motors and sensors
- Provides emergency stop capabilities based on sensor input

### Web Interface
The web interface is built with Flask, HTML, CSS, and JavaScript, running locally on the Raspberry Pi. It features:
- Real-time video streaming with corrosion detection overlays
- Interactive controls for manual robot operation
- Visualization of sensor data and robot status
- Historical inspection records with timestamp and location data
- User authentication for secure access
- Responsive design for use on various devices

## Results & Performance
[Detection accuracy metrics will be added later]

[Processing speed/latency numbers will be added later]

[Sample detection images with bounding boxes will be added later]

## Setup Instructions
[Hardware requirements will be added later]

[Software installation steps will be added later]

[Configuration process will be added later]

## Demo
[Link to video demonstration will be added later]

[Screenshots of the web interface will be added later]

## Technologies Used
- **Languages**: Python, C++ (Arduino), HTML/CSS/JavaScript
- **Computer Vision**: Roboflow API, YOLOv11, OpenCV
- **Hardware**: Raspberry Pi 5, Arduino Uno, Raspberry Pi Camera V2, ultrasonic sensors, motor drivers
- **Web Development**: Flask, Bootstrap
- **Data Storage**: SQLite for logging detection data
- **Containerization**: Docker for consistent deployment

## Key Skills Demonstrated
- Computer Vision
- Robotics & Embedded Systems Programming
- Real-Time Data Processing
- API Integration
- Web Development
- System Optimization
- Edge AI Deployment
