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

from dora_the_mug_finder_bringup.src.table_detection import PlaneDetection, PlaneTable, Table, Transform


def main():

    points_2D = np.array([(418, 247)], dtype="double")

    points_3D = np.array([(0.27847694189309097, 0.09962782553946638, 0.0)])
    	
    dist_coeffs = np.zeros((4,1))

    camera_matrix = np.array([ (36.26378216  , 0   ,  125.68539168),
                               (0  ,     36.76607372 ,142.49821147),
                               (0      ,     0    ,      1        )])


    success, rotation_vector, translation_vector = cv2.solvePnP(points_3D, points_2D, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
 
 
    nose_end_point2D, jacobian = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)



    files_path=f'{os.environ["DORA"]}'
    
    # Scene dataset paths
    filenames = []
    filenames.append (files_path + '/rgbd-scenes-v2/imgs/scene_02/00000-color.png')

    x = 0.27847694189309097
    y = 0.09962782553946638
    width = 0.09574153252956084
    height = 0.1062374627473996

    point_3d = np.array([x, y, 1], dtype=np.float32)

    # [63.00, 63.00]
    p1 = cv2.projectPoints(np.array([[point_3d]]), rvec, tvec, cammat, distortion)
    print(p1)

    for filename in filenames:
        image = cv2.imread(filename)
        for height in range(y,y+height):
            for width in range(x,x+width):
                image[height,width]=[255,255,255]

        cv2.imshow("Image", image)
        cv2.waitKey(0)


if __name__ == "__main__":
    main()