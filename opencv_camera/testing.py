from ultralytics import YOLO
import cv2
import math 
import homography as homo
import numpy as np

import utils 
# start webcam
cap = cv2.VideoCapture(0)
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 720

# cap.set(cv2.CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
# cap.set(cv2.CV_CAP_PROP_FRAME_HEIGHT, FRAME_WIDTH)

# model
model = YOLO("yolo-Weights/yolov8n.pt")
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

#black bag is our virtual plane = 37 x 37, 0,0 is top left corner
area_of_interest_2d = {'x1' : 0, 'y1': 0, 
                    'x2': 727, 'y2': 0, 
                    'x3': 0, 'y3': 727, 
                    'x4' : 727, 'y4': 727}

area_of_interest_2d = [[area_of_interest_2d["x1"], area_of_interest_2d["y1"]], 
                            [area_of_interest_2d["x2"], area_of_interest_2d["y2"]], 
                            [area_of_interest_2d["x3"], area_of_interest_2d["y3"]],
                            [area_of_interest_2d["x4"], area_of_interest_2d["y4"]]]

def get_coords_from_boxes(box):
    coord_map = {}
    x1, y1, x2, y2 = box.xyxy[0]
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
    coord_map['x1'], coord_map['y1'], coord_map['x2'], coord_map['y2'] = x1, y1, x2, y2 #store values into dictionary and pass back to user
    return coord_map

#add the hand gestures here, for now just use input
marked_area_of_interest = dict()
# marked_area_of_interest = utils.find_area_of_interest()
marked_area_of_interest = [[317, 170], [65, 322], [468, 452], [583, 216]]
# marked_area_of_interest = utils.find_area_of_interest()

while True:
    success, img = cap.read()
    results = model(img, stream=True)
    # when do we want to start using marking an image for coordinates
    for r in results:
        boxes = r.boxes
        H_matrix = homo.create_homography_matrix(marked_area_of_interest, area_of_interest_2d)
        pts = []
        for box in boxes:
            coord_map = get_coords_from_boxes(box)
            cv2.rectangle(img, (coord_map['x1'], coord_map['y1']), (coord_map['x2'], coord_map['y2']), (255, 0, 255), 3)
            confidence = math.ceil((box.conf[0]*100))/100
            # print("Confidence --->",confidence)
            # class name
            cls = int(box.cls[0])
            # print("Class name -->", classNames[cls])
            if classNames[cls] == 'sports ball' or 'person' or 'apple' or 'orange':
                #calculate lower centroid, save it to global np array to find equivalent homography points
                x1, y1, x2, y2 = coord_map['x1'], coord_map['y1'], coord_map['x2'], coord_map['y2']
                x = x2-x1
                y = y1
                if [x,y] not in pts:
                    pts.append([x,y, classNames[cls]])
            # object details
            org = [coord_map['x1'], coord_map['y1']]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2
            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)
        # if there are balls found in the array, process the found points with homography
        if pts:
            names = []
            pt_values = []
            for pt in pts:
                names.append([pt[2]])
                pt_values.append(pt[0:2])
            names_np = np.array(names)
            np_pts = np.array(pt_values)
            pts_hom = np.hstack((np_pts, np.ones((np_pts.shape[0], 1))))
            pts_transformed_hom = np.dot(H_matrix, pts_hom.T).T
            pts_transformed = pts_transformed_hom[:, :2] / pts_transformed_hom[:, 2:]
            pts_transformed = np.append(pts_transformed, names_np, axis =1)
            with open('/Users/johna/walden_temp_proj/ace_retriever/opencv_camera/distance.csv', 'w') as file:
                file.write(str(pts_transformed) + '\n')
            
            for pt in pts_transformed:
                if pt[2] == 'sports ball' or 'apple' or 'orange':
                    #send the node to arduino
                    #[['-132926.57251083312' '-10472.124226163163' 'person']]
                    x = int(pt[0])
                    y = int(pt[0])
                    if x < 0: #negative
                        x = x + 128
                    elif y < 0:
                        y = y + 128
                    node = [x, y]
                    sendNode = rf.write_array_to_arduino(node, port = 'COM6', baud_rate=115200)
                    if sendNode != True:
                        print("Didn't send to arduino")
                    
                print(pt)
                org = (int(float(pt[0])), int(float(pt[1])))
                # cv2.circle(img, str(pts_transformed), org, font, fontScale, color, thickness)
                cv2.circle(img, org, radius = 5, color = color)

            # src = np.array(marked_area_of_interest)
            # dst = np.array(black_bag_arr)
            # plan_view = cv2.warpPerspective(src, H_matrix, (dst.shape[1], dst.shape[0]))
    cv2.imshow('Webcam', img) 

    # if cv2.waitKey(1) == ord('p'):
    #     cv2.imshow('Plan View', plan_view)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()