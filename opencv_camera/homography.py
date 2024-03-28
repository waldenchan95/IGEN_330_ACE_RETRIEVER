import numpy as np
import cv2
import numpy as np
import matplotlib.pyplot as plt

#points of interest on original frame
#book = 13 x 20 cm 
# book coordinates from YOLO {'x1': 127, 'y1': 151, 'x2': 627, 'y2': 347}
# x5, y5 = 127, 151
# x6, y6 = 127, 347
# x7, y7 = 627, 151
# x8, y8 = 627, 347
# x9, y9 = 627-127, 347 - 151
# bottom middle value: x = 627-127, 151


#points of interest on virtual field
#book from photo
# x1, y1 = 2,2
# x2, y2 = 727, 6
# x3, y3 = 8, 643
# x4, y4 = 718, 641


#points of interest on virtual field
#black bag = 37 x 37
# x1, y1 = 0,0
# x2, y2 = 727, 0
# x3, y3 = 0, 727
# x4, y4 = 727, 727

def create_homography_matrix(pts_interest, pts_virtual):
    '''
    Develop your homography matrix based on a known 2D virtual map and equivalent coordinates from live feed.
    Returns the Homography matrix that you can use for YOLO
    '''
    np_array_pts_interest = np.array(pts_interest, dtype = np.float32)

    np_array_pts_virtual = np.array(pts_virtual, dtype = np.float32)

    H, _ = cv2.findHomography(np_array_pts_interest, np_array_pts_virtual)
    return H