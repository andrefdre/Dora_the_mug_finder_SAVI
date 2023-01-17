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

import rospy
from dora_the_mug_finder_msg.msg import Object , Point


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.center)

    files_path=f'{os.environ["DORA"]}'
    
    # Scene dataset paths
    filenames = []
    filenames.append (files_path + '/rgbd-scenes-v2/imgs/scene_03/00000-color.png')

    points = np.array([[center.x,center.y,center.z] for center in data.center],dtype = np.float64)
    bbox_3d =np.array( [[[data.corners[idx].x,data.corners[idx+1].y,data.corners[idx].z],[data.corners[idx+1].x,data.corners[idx].y,data.corners[idx].z]] for idx in range(0,len(data.corners),2)] ,dtype=np.float64)                    
    

    # Camera parameters
    center = [320 , 240]
    focal_length = 570.3
    MM_PER_M = 1000


    camera_matrix = np.array([[focal_length, 0,            center[0]],
                              [0,            focal_length, center[1]],
                              [0,            0,            1]])

    # Project the 3D points to the 2D image plane
    points_2d = cv2.projectPoints(points, np.identity(3), np.zeros(3), camera_matrix, None,)[0]

    bbox_2d = []

    for corners in bbox_3d:
        bbox_2d.append(cv2.projectPoints(corners, np.identity(3), np.zeros(3), camera_matrix, None,)[0])

    # Scale the points to image pixels
    points_2d = np.round(points_2d).astype(int)
    bbox_2d = np.round(bbox_2d).astype(int)

    # Blue color in BGR
    color = (0, 251, 255)
  
    # Line thickness of 2 px
    thickness = 2

    for filename in filenames:
        image = cv2.imread(filename)

        for idx,point_2d in enumerate(points_2d):
            image[point_2d[0][1]-2:point_2d[0][1]+2,point_2d[0][0]-2:point_2d[0][0]+2]=color
            image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
      
    cv2.imshow("window" , image)
    cv2.waitKey(1)


def main():

    rospy.init_node('image_extractor', anonymous=False)

    rospy.Subscriber("objects_publisher", Object, callback)

    rospy.spin()


if __name__ == "__main__":
    main()