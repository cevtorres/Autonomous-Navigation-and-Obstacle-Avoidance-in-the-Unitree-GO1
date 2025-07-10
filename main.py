from ultralytics import YOLO
import cv2
import numpy as np
import asyncio
import paho.mqtt.client as mqtt
from go1pylib import Go1, Go1Mode

# Load YOLO model
model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

async def main():
    mqtt_options = dict(port=1883, host='192.168.123.161', keepalive=60, protocol=mqtt.MQTTv311)
    robot = Go1(mqtt_options)
    robot.init()

    if not cap.isOpened():
        print("Could not open video device.")
        return

    print("Robot started scouting...")
    seen_cell_phone = False

    while not seen_cell_phone:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        height, width, _ = frame.shape
        frame_area = width * height

        # Check camera visibility
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        noise_level = np.std(gray)

        if brightness < 25 or noise_level < 10:
            print(f"Camera vision might be blocked (brightness={brightness:.2f}, std={noise_level:.2f}) — turning 180°.")
            robot.stop()
            await robot.turn_left(speed=0.4, duration_ms=1800)
            continue

        # Run YOLO detection
        results = model(frame, verbose=False)
        labels = results[0].names
        detected = [labels[int(cls)] for cls in results[0].boxes.cls]

        # Check for cell phone
        if "cell phone" in detected:
            print("Cell phone detected — entering DAMPING mode.")
            robot.stop()
            robot.set_mode(Go1Mode.DAMPING)
            break

        # Check for large obstacles (focus on center of frame)
        obstacle_detected = False
        for i, cls in enumerate(results[0].boxes.cls):
            label = labels[int(cls)]
            if label == "cell phone":
                continue

            box = results[0].boxes.xyxy[i]
            x1, y1, x2, y2 = map(int, box)
            box_area = (x2 - x1) * (y2 - y1)
            box_center_x = (x1 + x2) / 2

            is_center = width * 0.3 < box_center_x < width * 0.7
            is_big = box_area / frame_area > 0.18

            if is_center and is_big:
                print(f"Obstacle detected: '{label}' — turning 180° (centered & close).")
                robot.stop()
                await robot.turn_left(speed=0.4, duration_ms=1800)
                obstacle_detected = True
                break

        if not obstacle_detected:
            print("Clear path — walking forward.")
            robot.stop()
            robot.set_mode(Go1Mode.WALK)
            await robot.go_forward(speed=0.3, duration_ms=1500)

    cap.release()
    robot.disconnect()
    print("Finished.")

asyncio.run(main())
