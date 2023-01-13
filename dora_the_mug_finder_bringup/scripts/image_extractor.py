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
    filenames.append (files_path + '/rgbd-scenes-v2/imgs/scene_02/00000-color.png')

    width = 0.09574153252956084
    height = 0.1062374627473996

    tx = 0.951682
    ty = 0.837501
    tz = 1.22802 

    obj0=[ 0.26466404 ,-0.15494963 , 1.44848637]
    obj1=[-0.18527376 ,-0.22413577 , 1.60140775]
    obj2=[-0.32050713, -0.17328956 , 1.49553367]
    obj3=[ 0.24332523, -0.0247561  , 1.1732882 ]
    obj4=[-0.09786274 ,-0.12610798 , 1.12135535]


    #camera_matrix = cv2.initCameraMatrix2D([x,y,z],)

    points = np.array([obj0, obj1, obj2,obj3,obj4],dtype = np.float64)

    # Define the camera intrinsic matrix

    image_size = (640, 480)

    # camera_matrix = np.array([[focal_length, 0, principal_point[0]],
    #                           [0, focal_length, principal_point[1]],
    #                           [0, 0, 1]])

    focal_length = 29.1888  # Focal length of the camera
    principal_point = (640/2, 480/2)  # Principal point of the image (x, y)


    camera_matrix = np.array([[focal_length, 0, principal_point[0]],
                              [0, focal_length, principal_point[1]],
                              [0, 0, 1]])
 


    objpoints = []
    objpoints.append(np.array([obj0,obj1, obj2,obj3, obj4,obj4], dtype=np.float32))
    #print(objpoints)
    imgpoints = []
    imgpoints.append(np.array([[426,237],[259,186],[203,189],[413,192],[276,190],[276,190]], dtype=np.float32))

    #ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    #print(camera_matrix)

    # Define the camera extrinsic matrix

    #rotation_vector = np.array([-0.42699109,-2.63687232,0.25050348])
    #rotation_matrix = cv2.Rodrigues(rotation_vector)[0]

    rotation_matrix = np.array([[ 0.853223 ,0.14808 ,-0.500083],
                                [ 0.277499 ,-0.940751 ,0.194891],
                                [ -0.441594, -0.305058 ,-0.843762]])

    translation_matrix = np.array([[1.37552],
                                   [1.11348],
                                   [1.53762]])
   

    # Project the 3D points to the 2D image plane
    points_2d = cv2.projectPoints(points, rotation_matrix, translation_matrix, camera_matrix, None,)[0]
    # Scale the points to image pixels
    points_2d = np.round(points_2d).astype(int)

    # Print the result
    print(points_2d)



    for filename in filenames:
        image = cv2.imread(filename)

        for point_2d in points_2d:
            image[point_2d[0][0],point_2d[0][1],0:3]=255

        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        #ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], camera_matrix, None,flags=1)
        #print(f' {ret} \n {mtx} \n {dist} \n {rvecs} \n {tvecs}' )
      
    cv2.imshow("window" , image)
    cv2.waitKey(0)
if __name__ == "__main__":
    main()