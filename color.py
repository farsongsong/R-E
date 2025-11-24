import cv2
import numpy as np

# HSV 색상 범위 정의
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
lower_green = np.array([50, 100, 100])
upper_green = np.array([70, 255, 255])
COLOR_MAP = {
    'RED': "STOP (1)",
    'YELLOW': "CAUTION (2)",
    'GREEN': "GO (3)"
}
def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                              cv2.inRange(hsv, lower_red2, upper_red2))
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    count_red = np.sum(mask_red > 0)
    count_yellow = np.sum(mask_yellow > 0)
    count_green = np.sum(mask_green > 0)

    THRESHOLD = 500
    detected_color = None
    if count_red > THRESHOLD:
        detected_color = 'RED'
    elif count_yellow > THRESHOLD:
        detected_color = 'YELLOW'
    elif count_green > THRESHOLD:
        detected_color = 'GREEN'
    return detected_color, count_red, count_yellow, count_green
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    color, c_r, c_y, c_g = detect_color(frame)
    print(f"R: {c_r}, Y: {c_y}, G: {c_g}, Detected: {color}")
    cv2.imshow('Traffic Light Detector', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
print("finish")
