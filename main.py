from ultralytics import YOLO
import cv2
import numpy as np
import asyncio
import time
import paho.mqtt.client as mqtt
from go1pylib import Go1, Go1Mode

# Load YOLO model (small for better performance)
model = YOLO("yolov8s.pt")

# Open camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Could not open video device.")
    exit()

# Measure real FPS
print("⏱️ Measuring camera FPS...")
frame_count = 0
start_time = time.time()
while frame_count < 60:
    ret, frame = cap.read()
    if not ret:
        break
    frame_count += 1
end_time = time.time()
real_fps = max(1, round(frame_count / (end_time - start_time)))
print(f"Real camera FPS: {real_fps}")

# Reset camera
cap.release()
cap = cv2.VideoCapture(0)

# Set up video writer
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('robot_output.mp4', fourcc, real_fps, (width, height))

async def main():
    mqtt_options = dict(port=1883, host='192.168.123.161', keepalive=60, protocol=mqtt.MQTTv311)
    robot = Go1(mqtt_options)
    robot.init()

    print("🤖 Robot started scouting...")
    seen_cell_phone = False
    frame_counter = 0

    while not seen_cell_phone:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        #out.write(frame)
        frame_counter += 1
        print(f"Frame {frame_counter} recorded.")

        results = model(frame)
        labels = results[0].names
        detected = [labels[int(cls)] for cls in results[0].boxes.cls]

        height, width, _ = frame.shape
        frame_area = width * height
        obstacle_detected = False

        # Check for cell phone
        if "cell phone" in detected:
            print("Cell phone detected — entering DAMPING mode.")
            robot.set_mode(Go1Mode.DAMPING)
            break

        # Check if camera is blocked (e.g. black or blurry)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if np.std(gray) < 10:
            print("Camera vision is blocked — turning 180°.")
            robot.stop()
            await robot.turn_left(speed=0.4, duration_ms=1800)
            continue

        # Check for large close obstacles (not cell phone)
        for i, cls in enumerate(results[0].boxes.cls):
            label = labels[int(cls)]
            if label == "cell phone":
                continue

            box = results[0].boxes.xyxy[i]
            x1, y1, x2, y2 = map(int, box)
            box_area = (x2 - x1) * (y2 - y1)

            if box_area / frame_area > 0.18:
                print(f"Obstacle too close: {label} — turning 180°.")
                robot.stop()
                await robot.turn_left(speed=0.4, duration_ms=1800)
                obstacle_detected = True
                break

        if not obstacle_detected:
            print("Clear path — walking forward.")
            robot.set_mode(Go1Mode.WALK)
            await robot.go_forward(speed=0.3, duration_ms=1500)

    cap.release()
    out.release()
    robot.disconnect()
    print("Finished. Video saved as 'robot_output.mp4'.")

asyncio.run(main())
