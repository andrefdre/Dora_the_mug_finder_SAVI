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
    print(points)
    
    # bbox_3d = np.array( [[(AxisAlignedBoundingBox_0_1[min][0],AxisAlignedBoundingBox_0_1[max][1],AxisAlignedBoundingBox_0_1[min][2]) , (AxisAlignedBoundingBox_0_1[max][0],AxisAlignedBoundingBox_0_1[min][1],AxisAlignedBoundingBox_0_1[min][2])],
    #                     [(AxisAlignedBoundingBox_1_1[min][0],AxisAlignedBoundingBox_1_1[max][1],AxisAlignedBoundingBox_1_1[min][2]) , (AxisAlignedBoundingBox_1_1[max][0],AxisAlignedBoundingBox_1_1[min][1],AxisAlignedBoundingBox_1_1[min][2])],
    #                     [(AxisAlignedBoundingBox_2_1[min][0],AxisAlignedBoundingBox_2_1[max][1],AxisAlignedBoundingBox_2_1[min][2]) , (AxisAlignedBoundingBox_2_1[max][0],AxisAlignedBoundingBox_2_1[min][1],AxisAlignedBoundingBox_0_1[min][2])],
    #                     [(AxisAlignedBoundingBox_3_1[min][0],AxisAlignedBoundingBox_3_1[max][1],AxisAlignedBoundingBox_3_1[min][2]) , (AxisAlignedBoundingBox_3_1[max][0],AxisAlignedBoundingBox_3_1[min][1],AxisAlignedBoundingBox_0_1[min][2])],
    #                     [(AxisAlignedBoundingBox_4_1[min][0],AxisAlignedBoundingBox_4_1[max][1],AxisAlignedBoundingBox_4_1[min][2]) , (AxisAlignedBoundingBox_4_1[max][0],AxisAlignedBoundingBox_4_1[min][1],AxisAlignedBoundingBox_0_1[min][2])]])

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

    # Print the result
    print(points_2d)
    #print(bbox_2d)

    #print(bbox_2d[0][0][0])
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

    # obj0=[ 0.26466404 ,-0.15494963 , 1.44848637]
    # obj1=[-0.18527376 ,-0.22413577 , 1.60140775]
    # obj2=[-0.32050713, -0.17328956 , 1.49553367]
    # obj3=[ 0.24332523, -0.0247561  , 1.1732882 ]
    # obj4=[-0.09786274 ,-0.12610798 , 1.12135535]


    # obj0_1 = [ 0.26972195 ,-0.0676526  , 1.1738706 ]
    # AxisAlignedBoundingBox_0_1= { min: (0.103711, -0.133947, 1.07396), max: (0.379837, 0.0147909, 1.29347)}
    # obj1_1 = [-0.01215172 ,-0.23067668 , 1.63186339]
    # AxisAlignedBoundingBox_1_1= { min: (-0.124774, -0.293254, 1.50998), max: (0.111777, -0.189816, 1.73514)}
    # obj2_1 = [-0.19274 ,   -0.15293952  ,1.36978918]
    # AxisAlignedBoundingBox_2_1= { min: (-0.244499, -0.209246, 1.31729), max: (-0.125371, -0.102802, 1.42335)}
    # obj3_1 = [-0.45456324 ,-0.10366471 , 1.2772793 ]
    # AxisAlignedBoundingBox_3_1=  { min: (-0.544627, -0.170547, 1.18114), max: (-0.355203, -0.0545037, 1.36349)}
    # obj4_1 = [-0.06957649 , 0.02356743 , 0.9336359 ]
    # AxisAlignedBoundingBox_4_1= { min: (-0.144589, -0.0207741, 0.865681), max: (-0.00263497, 0.0830057, 1.01218)}


if __name__ == "__main__":
    main()