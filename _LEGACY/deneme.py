import cv2
import numpy as np
from collections import deque, Counter
import time

# ===== PID PARAMETRELERİ (Kendin optimize et!) =====
Kp = 0.5
Ki = 0.01
Kd = 0.1

previous_error = 0.0
integral = 0.0

# ===== KARAR SABİTLEME (SON 7 KARAR) =====
direction_history = deque(maxlen=7)

# ===== FPS HESAPLAMA =====
fps_start = time.time()
frame_count = 0

# ===== KAMERA BAŞLAT =====
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise IOError("[HATA] Kamera açılamadı!")

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

def preprocess_frame(frame):
    # CLAHE ile kontrast artırımı
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (5,5), 0)

    # Adaptif eşikleme ile keskin siyah-beyaz ayrımı
    mask = cv2.adaptiveThreshold(blurred, 255,
                                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV,
                                 11, 2)
    # Gürültü azaltmak için morfolojik kapama
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def line_following():
    global frame_count, fps_start

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Kamera görüntüsü alınamadı.")
            break

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        # ROI: Alt orta bölge (120x320 px)
        roi_h, roi_w = 120, 320
        roi_top = height - roi_h
        roi_bottom = height
        roi_left = (width - roi_w) // 2
        roi_right = roi_left + roi_w
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        mask = preprocess_frame(roi)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        direction = "CIZGI YOK"
        error = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])

                    roi_center = roi_w // 2
                    error = cx - roi_center

                    pid_output = calculate_pid(error)

                    # PID output kullanarak yön belirleme (eşik değerlerle)
                    if abs(error) < 20:
                        direction = "DUZ ILERLE"
                    elif error < -20:
                        direction = "SOL"
                    else:
                        direction = "SAG"

                    # Görsel olarak çizgi merkezi
                    cv2.circle(roi, (cx, cy), 7, (0,255,0), -1)
                    cv2.line(roi, (roi_center, 0), (roi_center, roi_h), (255,0,0), 2)

        stable_dir = get_stable_direction(direction)

        # FPS hesaplama
        frame_count += 1
        if frame_count >= 10:
            fps = frame_count / (time.time() - fps_start)
            fps_start = time.time()
            frame_count = 0
        else:
            fps = 0

        # Ana görüntü üzerine yazılar
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255,0,0), 2)
        cv2.putText(frame, f"YON: {stable_dir}", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(frame, f"ERROR: {error}", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)

        # Gösterimler
        cv2.imshow("Line Following", frame)
        cv2.imshow("ROI Mask", mask)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
