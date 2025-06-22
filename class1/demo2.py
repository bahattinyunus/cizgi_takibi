import cv2
import numpy as np
from collections import deque, Counter
import time

# PID parametreleri
Kp, Ki, Kd = 0.4, 0.0, 0.15
previous_error = 0
integral = 0

# Karar sabitleme için history
direction_history = deque(maxlen=7)

# Kamera başlatma
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[HATA] Kamera açılamadı.")
    exit()

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def adaptive_threshold(hsv_roi):
    # Otomatik adaptif eşik için histogram analizinden V kanalı medianı alıp ayarla
    v_channel = hsv_roi[:, :, 2]
    median_v = np.median(v_channel)
    lower_v = max(0, median_v - 50)
    upper_v = min(255, median_v + 50)
    return (0, 0, 0), (180, 255, int(upper_v))  # Siyah için dinamik üst sınır

def calculate_pid(error):
    global previous_error, integral
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

def draw_pid_output(frame, output, position=(30, 100)):
    # PID çıktısını grafiksel gösterim (yön ve şiddet)
    length = int(np.clip(abs(output)*2, 0, 100))  # Kuvveti ölçeklendir
    color = (0, 255, 0) if output >= 0 else (0, 0, 255)
    cv2.line(frame, position, (position[0]+length if output > 0 else position[0]-length, position[1]), color, 6)
    cv2.putText(frame, f"PID Output: {output:.2f}", (position[0], position[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def line_following():
    global previous_error, integral

    frame_count = 0
    start_time = time.time()
    
    # Başlangıçta ROI parametreleri
    roi_height = 150
    roi_width = 200
    roi_top_offset = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Frame alınamadı.")
            break

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape

        # ROI'yi dinamik kaydırıyoruz, çizgi hareketine göre
        roi_top = height - roi_height - roi_top_offset
        roi_bottom = height - roi_top_offset
        roi_left = width // 2 - roi_width // 2
        roi_right = width // 2 + roi_width // 2
        roi_center_x = roi_width // 2

        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        # Adaptive threshold
        lower_black, upper_black = adaptive_threshold(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV))
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, lower_black, upper_black)

        # Kenar tespiti
        edges = cv2.Canny(mask, 50, 150)
        kernel = np.ones((5, 5), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Kontur bulma
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        direction = "ÇİZGİ BULUNAMADI"

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                error = cx - roi_center_x
                pid_output = calculate_pid(error)

                # ROI kaydırma mantığı (çizgi çok üstte ise ROI yukarı kayar)
                if cy < roi_height // 3 and roi_top_offset < height - roi_height:
                    roi_top_offset += 10
                elif cy > 2 * roi_height // 3 and roi_top_offset > 0:
                    roi_top_offset -= 10

                small_thresh = 25
                medium_thresh = 60

                if abs(error) <= small_thresh:
                    direction = "DÜZ İLERLE"
                elif abs(error) <= medium_thresh:
                    direction = "SAĞA KAY" if error > 0 else "SOLA KAY"
                else:
                    direction = "SAĞA DÖN" if error > 0 else "SOLA DÖN"

                cv2.circle(roi, (cx, cy), 7, (0, 255, 0), -1)
                draw_pid_output(frame, pid_output)
            else:
                direction = "ÇİZGİ HESAPLANAMADI"
        else:
            direction = "ÇİZGİ BULUNAMADI"

        stable_direction = get_stable_direction(direction)

        # ROI ve metin gösterimi
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 2)
        cv2.putText(frame, f"YÖN: {stable_direction}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        # FPS hesaplama ve gösterim
        frame_count += 1
        if frame_count >= 10:
            end_time = time.time()
            fps = frame_count / (end_time - start_time)
            cv2.putText(frame, f"FPS: {fps:.1f}", (width - 150, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            frame_count = 0
            start_time = time.time()

        cv2.imshow("Çizgi Takibi", frame)
        cv2.imshow("Maske", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
