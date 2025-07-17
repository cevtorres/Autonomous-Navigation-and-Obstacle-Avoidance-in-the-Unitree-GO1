# Autonomous-Navigation-and-Obstacle-Avoidance-in-the-Unitree-GO1

## Summary
This project enables the Unitree GO1 robot to navigate autonomously using real-time object detection with YOLOv8. The robot walks forward when the path is clear, avoids nearby obstacles, and stops when a cell phone is detected. It also records a video of the entire session from the robot's camera.

## Features
Real-time object detection using YOLOv8s

Obstacle avoidance based on bounding box size and proximity

Records a video of the camera feed during operation

Performs incremental turns to reassess the environment when blocked

## Requirements
```bash
pip install ultralytics opencv-python numpy paho-mqtt
```
## How It Works
The program connects to the GO1 robot and initializes the YOLOv8 object detection model.

It starts reading frames from the camera in real-time.

If the camera is blocked or an obstacle is detected too close, the robot turns slightly and reassesses.

If target is detected, the robot enters DAMPING mode and stops.

If the path is clear, the robot walks forward briefly.

The process continues until target is seen.

## How To Run
```bash
python3 main.py
```



