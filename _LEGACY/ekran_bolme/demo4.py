import cv2
import numpy as np
from collections import deque, Counter
import time

# PID parametreleri (kendin ayarla sonra)
Kp, Ki, Kd = 0.5, 0.001, 0.2
previous_error = 0
integral = 0

# Karar sabitleme için history (yumuşak, ani yön değişimini engeller)
direction_history = deque(maxlen=7)

# FPS hesaplama için değişkenler
fps_start_time = time.time()
fps_frame_count = 0

# Kamera açma (mümkünse yüksek çözünürlük, yoksa ayarla)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("[HATA] Kamera açılamadı!")
    exit(1)

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def calculate_pid(error):
    global previous_error, integral
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

def preprocess_mask(frame):
    # CLAHE ile kontrast arttırma
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    
    # Blur ile gürültü azaltma
    blurred = cv2.GaussianBlur(enhanced, (5,5), 0)
    
    # Adaptive Threshold: değişen ışık koşullarına karşı dayanıklı
    mask = cv2.adaptiveThreshold(blurred, 255, 
                                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, 15, 7)
    
    # Morphology ile boşlukları doldur, çizgiyi kalınlaştır
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    return mask

def line_following():
    global fps_start_time, fps_frame_count

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Görüntü alınamadı!")
            break

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        # ROI: alt orta kısmı al, burası genelde çizgi olur
        roi_top = height - 150
        roi_bottom = height
        roi_left = width//2 - 150
        roi_right = width//2 + 150
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        # Maskeyi hazırla
        mask = preprocess_mask(roi)

        # Kontur bul
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        direction = "CIZGI YOK"
        error = 0

        if contours:
            # En büyük konturu bul
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                roi_center_x = (roi_right - roi_left) // 2
                error = cx - roi_center_x

                # PID ile düzeltilmiş hata
                pid_output = calculate_pid(error)

                # Basit yönlendirme
                if abs(error) < 20:
                    direction = "DUZ ILERLE"
                elif error < -20:
                    direction = "SOL"
                else:
                    direction = "SAG"

                # ROI içinde çizgiyi göster
                cv2.circle(roi, (cx, cy), 7, (0, 255, 0), -1)
                cv2.drawContours(roi, [largest_contour], -1, (0,255,0), 2)
            else:
                direction = "CIZGI YOK"
        else:
            direction = "CIZGI YOK"

        stable_direction = get_stable_direction(direction)

        # FPS hesapla
        fps_frame_count += 1
        if fps_frame_count >= 10:
            fps_end_time = time.time()
            fps = fps_frame_count / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_frame_count = 0
        else:
            fps = 0

        # Ana ekranda gösterimler
        cv2.putText(frame, f"YON: {stable_direction}", (30, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        cv2.putText(frame, f"FPS: {fps:.2f}", (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

        # ROI çerçevesi
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 3)

        cv2.imshow("Line Following", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
