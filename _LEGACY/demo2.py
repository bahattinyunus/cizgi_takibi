import cv2
import numpy as np
from collections import deque, Counter
import time
import sys

# PID parametreleri
Kp, Ki, Kd = 0.4, 0.0, 0.1
previous_error = 0
integral = 0

# Karar sabitleme
direction_history = deque(maxlen=7)

# Kamera başlat
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("[HATA] Kamera açılamadı.")
    sys.exit(1)

# FPS hesaplama
fps_start = time.time()
frame_count = 0

def calculate_pid(error):
    global previous_error, integral
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def line_following():
    global frame_count, fps_start
    kernel = np.ones((5, 5), np.uint8)
    lost_counter = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Görüntü alınamadı.")
            break

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        # ROI bölgesi
        roi_top, roi_bottom = height - 120, height
        roi_left, roi_right = width // 2 - 150, width // 2 + 150
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]
        roi_center_x = (roi_right - roi_left) // 2

        # Görüntü işleme
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        blur = cv2.GaussianBlur(enhanced, (5, 5), 0)
        mask = cv2.adaptiveThreshold(blur, 255,
                                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 11, 3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Kontur analizi
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        direction = "CIZGI YOK"
        delta_x = 0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                delta_x = cx - roi_center_x

                output = calculate_pid(delta_x)

                if abs(delta_x) < 25:
                    direction = "DUZ ILERLE"
                elif delta_x < -25:
                    direction = "SOL"
                else:
                    direction = "SAG"

                cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)
                lost_counter = 0
            else:
                direction = "MERKEZ HATASI"
        else:
            lost_counter += 1
            direction = "CIZGI YOK"

        stable_dir = get_stable_direction(direction)

        # FPS hesaplama
        frame_count += 1
        if frame_count >= 10:
            fps = frame_count / (time.time() - fps_start)
            fps_start = time.time()
            frame_count = 0
        else:
            fps = 0

        # Arayüz çizimleri
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 2)
        color_map = {"DUZ ILERLE": (0, 255, 0), "SOL": (0, 255, 255), "SAG": (255, 255, 0), "CIZGI YOK": (0, 0, 255)}
        cv2.putText(frame, f"YON: {stable_dir}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    color_map.get(stable_dir, (255, 255, 255)), 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

        if lost_counter > 30:
            cv2.putText(frame, "CIZGI KAYBOLDU!", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # Gösterim
        cv2.imshow("Line Following", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()