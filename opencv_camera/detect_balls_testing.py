from ultralytics import YOLO
import cv2
import math 
import numpy as np
# start webcam
cap = cv2.VideoCapture(1)
WIDTH  = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))   # float `width`
HEIGHT = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))  # float `height`
import time
# cap.set(3, 640)
# cap.set(4, 480)

# model
model = YOLO("yolo-Weights/yolov8n.pt")
from ..python import rrt_star as rrt
from ..python import rf_communication as rf

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
names_dict = {i: name for i, name in enumerate(model.names)}


def get_coords_from_boxes(box):
    coord_map = {}
    x1, y1, x2, y2 = box.xyxy[0]
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
    coord_map['x1'], coord_map['y1'], coord_map['x2'], coord_map['y2'] = x1, y1, x2, y2 #store values into dictionary and pass back to user
    return coord_map



while True:
    success, img = cap.read()
    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        pts = []
        for box in boxes:
            coord_map = get_coords_from_boxes(box)
            cv2.rectangle(img, (coord_map['x1'], coord_map['y1']), (coord_map['x2'], coord_map['y2']), (255, 0, 255), 3)
            # print("Confidence --->",confidence)
            if classNames[cls] == 'sports ball' or 'person' or 'apple' or 'orange':
                #calculate lower centroid, save it to global np array to find equivalent homography points
                x1, y1, x2, y2 = coord_map['x1'], coord_map['y1'], coord_map['x2'], coord_map['y2']
                x = x2-x1
                y = y1
                if [x,y] not in pts:
                    pts.append([x,y, classNames[cls]])
            # class name
            cls = int(box.cls[0])
            # print("Class name -->", classNames[cls])
            # object details
            org = [coord_map['x1'], coord_map['y1']]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)
            #cv2.putText(img, str(distances), org, font, fontScale, color, thickness)
        if pts:
            names = []
            pt_values = []
            for pt in pts:
                names.append([pt[2]])
                pt_values.append(pt[0:2])
            names_np = np.array(names)
            np_pts = np.array(pt_values)
            pts_transformed = np.append(pts_transformed, names_np, axis =1)
            with open('/Users/johna/walden_temp_proj/ace_retriever/opencv_camera/distance.csv', 'w') as file:
                file.write(str(pts_transformed) + '\n')
            for pt in pts_transformed:
                org = (int(float(pt[0])), int(float(pt[1])))
                cv2.circle(img, str(pts_transformed), org, font, fontScale, color, thickness)
            if pts_transformed:
                start = [0,0]
                obstacles = []
                goals = []
                for pts in pts_transformed:
                    if pts[2] == 'person':
                        obstacles.append([int(pts[0]), int(pts[1])])
                    elif pts[2] == 'sports ball'  or 'apple' or 'orange':
                        obstacles.append([int(pts[0]), int(pts[1])])
                
                #if multiple balls, go to the one closer to start

                if len(goals) > 1:
                    closest_ball = [math.inf, math.inf]
                    for ball in goals:
                        if ball[0] - start[0] <= closest_ball[0] and ball[1] - start[1] <= closest_ball[1]:
                            closest_ball[0], closest_ball[1] = ball[0], ball[1]
                else:
                    closest_ball = goals
                
                width_scaled = 256
                height_scaled = WIDTH/HEIGHT * width_scaled

                bestPath = rrt.RRT(start = start, goal = closest_ball, obstacles = obstacles, width = width_scaled, height=height_scaled)
                time.sleep(5)
                with open('/Users/johna/walden_temp_proj/ace_retriever/opencv_camera/path.csv', 'w') as file:
                    file.write(str(bestPath) + '\n')
                
                #ensure that bestPath is no more than 8 nodes
                if len(bestPath) > 8:
                    while len(bestPath) > 8:
                        del bestPath[(len(bestPath)-1)/2]

                #write the path to Arduino
                time.sleep(5)
                sentData = rf.write_array_to_arduino(bestPath)
                if sentData != True:
                    print("Didn't send to Arduino")


    cv2.imshow('Webcam', img) 
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()