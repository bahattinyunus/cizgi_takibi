import cv2
import numpy as np
import time
from collections import deque, Counter

# --- PID PARAMETRELERİ ---
Kp, Ki, Kd = 0.45, 0.001, 0.2
integral_limit = 1000

# --- GLOBAL DEĞİŞKENLER ---
previous_error = 0.0
integral = 0.0
direction_history = deque(maxlen=7)

# --- KAMERA AYARLARI ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise RuntimeError("Kamera açılamadı!")

# --- YARDIMCI FONKSİYONLAR ---

def calculate_pid(error):
    global previous_error, integral

    integral += error
    # integral windup engelleme
    integral = max(min(integral, integral_limit), -integral_limit)
    
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative

    previous_error = error
    return output

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def preprocess_frame(frame):
    # 1. CLAHE ile kontrast artırma
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)

    # 2. Gaussian blur
    blur = cv2.GaussianBlur(enhanced, (7,7), 0)

    # 3. Adaptif threshold (çoklu ışık koşullarına uyum için)
    thresh = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11, 3
    )
    # 4. Morfolojik kapanış ile çizgi birleştirme
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    return closed

def find_line_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:  # Minimum alan filtreleme
            return largest
    return None

def line_following():
    global previous_error, integral

    fps_start = time.time()
    frame_count = 0
    fps = 0
    lost_line_counter = 0
    max_lost_frames = 30

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Görüntü alınamadı!")
            break

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        # --- ROI: alt 1/3 alan (dinamik yapabiliriz ama şimdilik sabit) ---
        roi = frame[int(height*2/3):height, :]

        # --- Ön işlem ---
        mask = preprocess_frame(roi)

        # --- Çizgi kontur bulma ---
        line_contour = find_line_contour(mask)
        direction = "CIZGI YOK"
        error = 0

        if line_contour is not None:
            lost_line_counter = 0
            M = cv2.moments(line_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Hata hesaplama (ROI merkezine göre)
                roi_center = width // 2
                error = cx - roi_center

                # PID kontrol
                control = calculate_pid(error)

                # Yön belirleme için eşikler
                threshold = 30
                if abs(error) < threshold:
                    direction = "DUZ ILERLE"
                elif error < -threshold:
                    direction = "SOL"
                else:
                    direction = "SAG"

                # Çizgi ve merkez noktalarını çiz
                cv2.drawContours(roi, [line_contour], -1, (0,255,0), 3)
                cv2.circle(roi, (cx, cy), 7, (0,0,255), -1)
                cv2.line(roi, (roi_center, 0), (roi_center, roi.shape[0]), (255,0,0), 2)

        else:
            lost_line_counter += 1
            direction = "CIZGI KAYIP"
            # Eğer çizgi uzun süredir kayıtsa PID integral'i sıfırla ki saçma hareket etmesin
            if lost_line_counter > max_lost_frames:
                integral = 0
                previous_error = 0

        # Karar sabitleme
        stable_direction = get_stable_direction(direction)

        # --- Görsel arayüz ---
        cv2.putText(frame, f"YON: {stable_direction}", (30,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)
        cv2.putText(frame, f"HATA: {error}", (30,100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame, f"PID Output: {control:.2f}", (30,140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

        # FPS hesaplama
        frame_count += 1
        if frame_count >= 20:
            fps = frame_count / (time.time() - fps_start)
            fps_start = time.time()
            frame_count = 0
        cv2.putText(frame, f"FPS: {fps:.2f}", (frame.shape[1]-160, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2)

        # Ana frame ve ROI maskesi gösterimi
        cv2.imshow("ROV Line Following", frame)
        cv2.imshow("Processed Mask", mask)

        # Çıkış için 'q' tuşu
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
