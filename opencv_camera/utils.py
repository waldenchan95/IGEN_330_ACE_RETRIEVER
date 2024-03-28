import numpy as np
import cv2

# startPoint = False
# endPoint = False
#global rect, startPoint, endPoint

def on_mouse(event,x,y,flags,params):
        global rect, startPoint, endPoint
        # get mouse click
        if event == cv2.EVENT_LBUTTONDOWN:
            if startPoint == True and endPoint == True:
                startPoint = False
                endPoint = False
                rect = (0, 0, 0, 0)
            if startPoint == False:
                rect = (x, y, 0, 0)
                startPoint = True
            elif endPoint == False:
                rect = (rect[0], rect[1], x, y)
                endPoint = True

def left_click(event,x,y,flags,params):
        global coords
        # get mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:
            coords.append([x,y])

def find_rect_of_interest():
    global rect, startPoint, endPoint
    rect = (0,0,0,0)
    startPoint = False
    endPoint = False

    cap = cv2.VideoCapture(0)
    waitTime = 50

    #Reading the first frame
    (grabbed, frame) = cap.read()

    while(cap.isOpened()):

        (grabbed, frame) = cap.read()

        cv2.namedWindow('frame')
        cv2.setMouseCallback('frame', on_mouse)    

        #drawing rectangle
        if startPoint == True and endPoint == True:
            cv2.rectangle(frame, (rect[0], rect[1]), (rect[2], rect[3]), (255, 0, 255), 2)
            org1, org2 = (rect[0], rect[1]), (rect[2], rect[3])
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2
            cv2.putText(frame, str((rect[0], rect[1])), org1, font, fontScale, color, thickness)
            cv2.putText(frame, str((rect[2], rect[3])), org2, font, fontScale, color, thickness)
        cv2.imshow('frame',frame)

        key = cv2.waitKey(waitTime) 

        if key == 27:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

    cap.release()
    cv2.destroyAllWindows()
    return rect

def find_area_of_interest():
    global coords
    coords = []
    cap = cv2.VideoCapture(0)
    waitTime = 50

    #Reading the first frame
    (grabbed, frame) = cap.read()

    while(cap.isOpened()):

        (grabbed, frame) = cap.read()

        cv2.namedWindow('frame')
        cv2.setMouseCallback('frame', left_click)    
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (255, 0, 0)
        thickness = 2
        for i in range(len(coords)):
            org = coords[i]
            cv2.putText(frame, str((coords[i][0], coords[i][1])), org, font, fontScale, color, thickness)
        
        cv2.imshow('frame',frame)

        key = cv2.waitKey(waitTime) 
        if key == 27:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

        if cv2.waitKey(1) & 0xFF == ord('r'): 
            coords.clear()
            find_area_of_interest()

    cap.release()
    cv2.destroyAllWindows()
    return coords


# print(find_area_of_interest())

