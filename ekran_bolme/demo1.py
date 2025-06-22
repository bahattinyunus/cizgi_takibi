import cv2
import numpy as np
from collections import deque, Counter

# PID parametreleri
Kp = 0.4
Ki = 0.0
Kd = 0.15
previous_error = 0
integral = 0

# Karar sabitleme
direction_history = deque(maxlen=5)

# Kamera başlat
cap = cv2.VideoCapture(0)

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def line_following():
    global previous_error, integral

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape

        # 3x3 grid için hazırlık
        grid_h = height // 3
        grid_w = width // 3
        weights = [[0]*3 for _ in range(3)]

        # Maskeleme
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 80))

        for i in range(3):
            for j in range(3):
                y1 = i * grid_h
                y2 = (i + 1) * grid_h
                x1 = j * grid_w
                x2 = (j + 1) * grid_w

                cell = mask[y1:y2, x1:x2]
                black_pixel_count = cv2.countNonZero(cell)
                weights[i][j] = black_pixel_count

                # Görsel çizimler
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)
                cv2.putText(frame, str(black_pixel_count), (x1 + 5, y1 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Karar mekanizması
        bottom_row = weights[2]
        middle_row = weights[1]

        left = bottom_row[0] + middle_row[0]
        center = bottom_row[1] + middle_row[1]
        right = bottom_row[2] + middle_row[2]

        if max(left, center, right) < 500:
            direction = "CIZGI YOK"
        elif center >= max(left, right):
            direction = "DUZ ILERLE"
        elif left > right:
            direction = "SOL"
        else:
            direction = "SAG"

        stable_direction = get_stable_direction(direction)

        cv2.putText(frame, f"YON: {stable_direction}", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Gösterim
        cv2.imshow("Line Following", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    line_following()
