import cv2
import numpy as np
from collections import deque, Counter

# PID parametreleri (İstersen buradan ayarla)
Kp = 0.4
Ki = 0.0
Kd = 0.15
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
    # En çok tekrar eden yönü döndürür (gürültüyü azaltmak için)
    return Counter(direction_history).most_common(1)[0][0]

def line_following():
    global previous_error, integral

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[HATA] Frame alınamadı.")
            break

        frame = cv2.flip(frame, 1)  # Aynalama
        height, width, _ = frame.shape

        # ROI (bölge) tanımı - alt orta bölgeye odaklanıyoruz
        roi_height = 150
        roi_width = 200
        roi_top = height - roi_height
        roi_bottom = height
        roi_left = width // 2 - roi_width // 2
        roi_right = width // 2 + roi_width // 2
        roi_center_x = roi_width // 2

        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        # Görüntüyü HSV'ye çevir, siyah çizgiyi maskele
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, (0, 0, 0), (180, 255, 80))

        # Canny ile kenar tespiti ve morfolojik kapanış (gürültü azaltma)
        edges = cv2.Canny(mask, 50, 150)
        kernel = np.ones((5,5), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Kontur bulma
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        direction = "ÇİZGİ BULUNAMADI"
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # X koordinatı (çizgi merkezi)
                cy = int(M["m01"] / M["m00"])  # Y koordinatı (çizgi merkezi)

                # Hata hesaplama: Çizginin ROI ortasından sapması
                error = cx - roi_center_x
                integral += error
                derivative = error - previous_error
                output = Kp * error + Ki * integral + Kd * derivative
                previous_error = error

                # Hata değerine göre yön belirleme
                small_thresh = 25
                medium_thresh = 60

                if abs(error) <= small_thresh:
                    direction = "DÜZ İLERLE"
                elif abs(error) <= medium_thresh:
                    direction = "SAĞA KAY" if error > 0 else "SOLA KAY"
                else:
                    direction = "SAĞA DÖN" if error > 0 else "SOLA DÖN"

                # Görsel işaretleme
                cv2.circle(roi, (cx, cy), 7, (0, 255, 0), -1)
            else:
                direction = "ÇİZGİ HESAPLANAMADI"
        else:
            direction = "ÇİZGİ BULUNAMADI"

        # Karar stabilizasyonu (gürültü önleme)
        stable_direction = get_stable_direction(direction)

        # ROI ve yön metni çizimi
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 2)
        cv2.putText(frame, f"YÖN: {stable_direction}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        # Ekranda gösterim
        cv2.imshow("Çizgi Takibi", frame)
        cv2.imshow("Maske", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
