#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------

import math
from more_itertools import locate
from math import floor, sqrt
from copy import deepcopy
import open3d as o3d
import numpy as np
import glob
import os
import cv2


def main():
    files_path=f'{os.environ["DORA"]}'
    
    # Scene dataset paths
    filenames = []
    filenames.append (files_path + '/rgbd-scenes-v2/imgs/scene_03/00000-color.png')


    obj0=[ 0.26466404 ,-0.15494963 , 1.44848637]
    obj1=[-0.18527376 ,-0.22413577 , 1.60140775]
    obj2=[-0.32050713, -0.17328956 , 1.49553367]
    obj3=[ 0.24332523, -0.0247561  , 1.1732882 ]
    obj4=[-0.09786274 ,-0.12610798 , 1.12135535]


    obj0_1 = [ 0.26972195 ,-0.0676526  , 1.1738706 ]
    obj1_1 = [-0.01215172 ,-0.23067668 , 1.63186339]
    obj2_1 = [-0.19274 ,   -0.15293952  ,1.36978918]
    obj3_1 = [-0.45456324 ,-0.10366471 , 1.2772793 ]
    obj4_1 = [-0.06957649 , 0.02356743 , 0.9336359 ]


    points = np.array([obj0_1, obj1_1, obj2_1,obj3_1,obj4_1],dtype = np.float64)

    # Camera parameters
    center = [320 , 240]
    imw=640
    imh=480
    focal_length = 570.3
    MM_PER_M = 1000


    camera_matrix = np.array([[focal_length, 0,            center[0]],
                              [0,            focal_length, center[1]],
                              [0,            0,            1]])

    # Project the 3D points to the 2D image plane
    points_2d = cv2.projectPoints(points, np.identity(3), np.zeros(3), camera_matrix, None,)[0]
    # Scale the points to image pixels
    points_2d = np.round(points_2d).astype(int)

    # Print the result
    print(points_2d)

    for filename in filenames:
        image = cv2.imread(filename)

        for point_2d in points_2d:
            image[point_2d[0][1],point_2d[0][0],0:3]=255
      
    cv2.imshow("window" , image)
    cv2.waitKey(0)
if __name__ == "__main__":
    main()