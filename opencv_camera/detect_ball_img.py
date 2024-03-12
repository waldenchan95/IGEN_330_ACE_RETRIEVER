from ultralytics import YOLO
from ultralytics.solutions import distance_calculation
import cv2
import math 
# start webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# model
model = YOLO("yolo-Weights/yolov8n.pt")

# object classes
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]
# classNames = ["person", "sports ball", "pottedplant", "bed", "chair"
#               "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone"
#               ]


def get_coords_from_boxes(box):
    coord_map = {}
    x1, y1, x2, y2 = box.xyxy[0]
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
    coord_map['x1'], coord_map['y1'], coord_map['x2'], coord_map['y2'] = x1, y1, x2, y2 #store values into dictionary and pass back to user
    return coord_map


dist_obj = distance_calculation.DistanceCalculation()
while True:
    success, img = cap.read()
    results = model(img, stream=True)
    tracker = model.track(img, persist = True)
    distances = dist_obj.calculate_distance(tracker)
    # coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            # x1, y1, x2, y2 = box.xyxy[0]
            # x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
            # print (x1, y1, x2, y2)
            # put box in cam
            coord_map = get_coords_from_boxes(box)
            cv2.rectangle(img, (coord_map['x1'], coord_map['y1']), (coord_map['x2'], coord_map['y2']), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            # print("Confidence --->",confidence)

            # class name
            cls = int(box.cls[0])
            # print("Class name -->", classNames[cls])

            # object details
            org = [coord_map['x1'], coord_map['y1']]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, classNames[cls]+distances, org, font, fontScale, color, thickness)

    cv2.imshow('Webcam', img) 
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()