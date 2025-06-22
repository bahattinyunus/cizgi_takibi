import cv2
import numpy as np
from collections import deque, Counter
import time

# --- Kamera ayarları ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# --- PID parametreleri ---
Kp, Ki, Kd = 0.4, 0.0, 0.1
previous_error = 0
integral = 0

# --- Yön sabitleme ---
direction_history = deque(maxlen=7)

def get_stable_direction(direction):
    direction_history.append(direction)
    return Counter(direction_history).most_common(1)[0][0]

# FPS Takibi
fps_start = time.time()
frame_count = 0

def line_following():
    global previous_error, integral, frame_count, fps_start

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Kamera verisi alınamadı.")
            break

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        # --- ROI alanı (alt merkez) ---
        roi_top = height - 120
        roi_bottom = height
        roi_left = width // 2 - 150
        roi_right = width // 2 + 150
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]
        roi_center_x = (roi_right - roi_left) // 2

        # --- Görüntü iyileştirme ---
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        blur = cv2.GaussianBlur(enhanced, (5, 5), 0)

        # --- Threshold işlemi ---
        mask = cv2.adaptiveThreshold(blur, 255,
                                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 11, 3)

        # --- Morfolojik işlemler ---
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # --- Kontur analizi ---
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

                # PID Hesaplama
                error = delta_x
                integral += error
                derivative = error - previous_error
                previous_error = error

                pid_output = Kp * error + Ki * integral + Kd * derivative

                # Yön kararı
                if abs(delta_x) < 25:
                    direction = "DUZ ILERLE"
                elif delta_x < -25:
                    direction = "SOL"
                elif delta_x > 25:
                    direction = "SAG"

                cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)
            else:
                direction = "MERKEZ YOK"
        else:
            direction = "CIZGI YOK"

        # Stabil karar
        stable_dir = get_stable_direction(direction)

        # FPS
        frame_count += 1
        if frame_count >= 10:
            fps = frame_count / (time.time() - fps_start)
            fps_start = time.time()
            frame_count = 0
        else:
            fps = 0

        # ROI işaretleme
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 2)

        # Debug text
        cv2.putText(frame, f"YON: {stable_dir}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Gösterimler
        cv2.imshow("Line Following", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if _name_ == "_main_":
    line_following()