import cv2
import numpy as np

COLOR_RANGES_HSV = {
    "red": [(0, 50, 10), (10, 255, 255)],
    "orange": [(10, 50, 10), (25, 255, 255)],
    "yellow": [(25, 50, 10), (35, 255, 255)],
    "green": [(35, 50, 10), (80, 255, 255)],
    "cyan": [(80, 50, 10), (100, 255, 255)],
    "blue": [(100, 50, 10), (130, 255, 255)],
    "purple": [(130, 50, 10), (170, 255, 255)],
    "red ": [(170, 50, 10), (180, 255, 255)]
}
#Lets use HSV colour coding for our purposes so that it can handle more lighting situations
cap = cv2.VideoCapture(0)

def detectCirclesWithDp(frame, dp=1.9):
    blurred = cv2.medianBlur(frame, 25)
    grayMask = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    # cannyMask = cv2.Canny(grayMask, 50, 240)
    return cv2.HoughCircles(grayMask, cv2.HOUGH_GRADIENT, dp, 30, param1=100, param2=80, minRadius=0, maxRadius=0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    detected_circles = detectCirclesWithDp(frame)
    if detected_circles is not None:
        detected_circles = np.uint16(np.around(detected_circles))
        for i in detected_circles[0, :]:
            a, b, r = i[0], i[1], i[2]
            cv2.circle(frame, (a, b), r, (0, 255, 0), 2) 
  
        # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)
    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     for circle in circles[0,:]:
    #         lower_limit = np.array([20, 100, 100], dtype=np.uint8)
    #         upper_limit = np.array([30, 255, 255], dtype=np.uint8)
    #         mask = cv2.inRange(hsv_frame, lower_limit, upper_limit)
    #         cv2.circle(frame, (circle[0], circle[1]), circle[2], (250,0,0), 1)
    #         cv2.circle(frame, (circle[0], circle[1]), 2, (250,0,0), 2)
    #         cv2.putText(frame, 'color', (int(circle[0] + 40), int(circle[1] + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
    #                         (250,0,0))

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()