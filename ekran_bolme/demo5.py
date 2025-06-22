import cv2
import numpy as np
from collections import deque, Counter
import time

# PID parametreleri, sadece hata analizi için kaldı (motor kontrolü yapmayacaksın diye kullanma zorunlu değil)
Kp, Ki, Kd = 0.6, 0.001, 0.25
previous_error = 0
integral = 0

# Son 10 yönün kararlı karar için hafızası
direction_history = deque(maxlen=10)

# FPS takibi için sayaçlar
fps_start_time = time.time()
fps_frame_count = 0

# Kamera başlatılıyor
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("[HATA] Kamera açılamadı!")
    exit(1)

# Çizgi kaybolursa arama için sayaç
lost_counter = 0
LOST_THRESHOLD = 20  # kaç frame çizgi yoksa arama moduna geç

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def adaptive_threshold(gray_frame):
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray_frame)
    blurred = cv2.GaussianBlur(enhanced, (5,5), 0)
    return cv2.adaptiveThreshold(blurred, 255, 
                                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, 15, 7)

def preprocess_mask(roi):
    mask = adaptive_threshold(roi)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)
    return mask

def line_following():
    global fps_start_time, fps_frame_count, lost_counter

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Görüntü alınamadı!")
            break

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        # Dar ROI (alt orta) çizgi takip için
        roi_top = height - 150
        roi_bottom = height
        roi_left = width // 2 - 150
        roi_right = width // 2 + 150
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = preprocess_mask(gray_roi)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        direction = "CIZGI YOK"
        error = 0

        if contours:
            lost_counter = 0
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 500:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    roi_center_x = (roi_right - roi_left) // 2
                    error = cx - roi_center_x

                    if abs(error) < 15:
                        direction = "DUZ ILERLE"
                    elif error < -15:
                        direction = "SOL"
                    else:
                        direction = "SAG"

                    cv2.drawContours(roi, [largest_contour], -1, (0, 255, 0), 3)
                    cv2.circle(roi, (cx, cy), 7, (0, 255, 0), -1)
        else:
            lost_counter += 1
            if lost_counter > LOST_THRESHOLD:
                direction = "CIZGI KAYIP!"

        stable_direction = get_stable_direction(direction)

        # FPS hesaplama
        fps_frame_count += 1
        if fps_frame_count >= 15:
            fps_end_time = time.time()
            fps = fps_frame_count / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_frame_count = 0
        else:
            fps = 0

        # Ana ekranda bilgi yaz
        cv2.putText(frame, f"YON: {stable_direction}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 3)
        cv2.putText(frame, f"HATA: {error}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,200), 2)
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2)

        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 3)

        cv2.imshow("Line Following - ROV Mode", frame)
        cv2.imshow("Mask (ROI)", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("[BİTİYOR] Çıkılıyor...")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
