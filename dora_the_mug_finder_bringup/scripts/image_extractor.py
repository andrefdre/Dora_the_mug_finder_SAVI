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
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse
import sys
from scipy.spatial.transform import Rotation as R

import rospy
from dora_the_mug_finder_msg.msg import Object , Images


class Image:

    def __init__(self):
        self.figure = plt.figure("Cropped Images")
        self.figure.set_size_inches(10,5)
        self.cropped_images = Images()
        self.pub = rospy.Publisher('image_publisher', Images, queue_size=10)
        self.bridge = CvBridge()

    def callback(self,data):
        files_path=f'{os.environ["DORA"]}'
        
        # Pose dataset paths
        filename_pose = (files_path + '/rgbd-scenes-v2/pc/02.pose')
        

        # Scene dataset paths
        filenames = []
        filenames.append (files_path + '/rgbd-scenes-v2/imgs/scene_02/00233-color.png')
       
        points = np.array([[center.x,center.y,center.z] for center in data.center],dtype = np.float64)
        bbox_3d =np.array( [[[data.corners[idx].x,data.corners[idx+1].y+0.05,data.corners[idx].z],[data.corners[idx+1].x,data.corners[idx].y,data.corners[idx].z]] for idx in range(0,len(data.corners),2)] ,dtype=np.float64)                    
        
        # ????????????????????????????????????????????????????????????????

        self.matrix(filename_pose)
        # print('points: \n',points)
        # print(self.matrix_inv)
        for idx, p in enumerate(points):
            point = np.ones((1,4))
            point[:,0:3] = points[idx]
            point = np.transpose(point)
            point = np.dot(self.matrix_inv,point)
            points[idx,:]=np.transpose(point[0:3,:]) 
        
        # print('points 1 new: \n', points)  
        
        # ????????????????????????????????????????????????????????????????   

        # Camera parameters
        center = [320 , 240]
        focal_length = 570.3
        MM_PER_M = 1000


        camera_matrix = np.array([[focal_length, 0,            center[0]],
                                [0,            focal_length, center[1]],
                                [0,            0,            1]])

        # Project the 3D points to the 2D image plane
        points_2d = cv2.projectPoints(points, self.rot_matrix, self.trans_matrix, camera_matrix, None,)[0]
        print('points_2d: \n',points_2d)
        bbox_2d = []

        for corners in bbox_3d:
            bbox_2d.append(cv2.projectPoints(corners, self.rot_matrix, self.trans_matrix, camera_matrix, None,)[0])

        # Scale the points to image pixels
        points_2d = np.round(points_2d).astype(int)
        bbox_2d = np.round(bbox_2d).astype(int)

        # Blue color in BGR
        color = (0, 251, 255)
    
        # Line thickness of 2 px
        thickness = 2

        self.cropped_images = Images()
        for filename in filenames:
            image = cv2.imread(filename)
            image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

            for idx,point_2d in enumerate(points_2d):
                image[point_2d[0][1]-5:point_2d[0][1]+5,point_2d[0][0]-5:point_2d[0][0]+5]=color
                #image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
                cropped_image = image[bbox_2d[idx][1][0][1]:bbox_2d[idx][0][0][1],bbox_2d[idx][0][0][0]:bbox_2d[idx][1][0][0]]
                height ,width , _ = cropped_image.shape
                self.cropped_images.images.append(self.bridge.cv2_to_imgmsg(image, "passthrough"))
            
        self.pub.publish(self.cropped_images)


    def matrix(self,filename):
        
        with open(filename, "r") as pose:
            file_pose = pose.readlines()
            vector_pose_str = file_pose[233]

            vector_pose_array = vector_pose_str.split(' ')
            
            # rotation matrix
            r = R.from_quat([vector_pose_array[1], vector_pose_array[2], vector_pose_array[3], vector_pose_array[0]])
            self.rot_matrix = r.as_matrix()
            
            # translation matrix
            self.trans_matrix = np.array([vector_pose_array[4], vector_pose_array[5], vector_pose_array[6]], dtype=np.float32)
            # self.trans_matrix = np.vstack(trans_matrix)

            # homogeneous transformation matrix (4,4) 
            l = 4
            self.matrix = np.zeros((l,l))
            self.matrix[0:3,0:3] = self.rot_matrix
            self.matrix[0:3,3] = self.trans_matrix
            self.matrix[3,3] = 1

            # inverse matrices
            self.matrix_inv = np.linalg.inv(self.matrix)
            self.rot_matrix_inv, self.trans_matrix_inv = np.hsplit(np.array(self.matrix_inv), [l-1])
            self.rot_matrix_inv = self.rot_matrix_inv[0:3,:]
            self.trans_matrix_inv = self.trans_matrix_inv[0:3,:]

            # prints
            # print('rot matrix: \n', self.rot_matrix)
            # print("trans:\n", self.trans_matrix)
            # print("matrix:\n", self.matrix)
            
            # print("matrix inv:\n", self.matrix_inv)
            # print('rot inv: \n', self.rot_matrix_inv)
            # print('trans inv:\n', self.trans_matrix_inv)

          

    def draw(self):
        plt.clf()
        for image_idx,cropped_image in enumerate(self.cropped_images.images,start=1):
            ax = self.figure.add_subplot(2,5,image_idx) # define a 5 x 5 subplot matrix
            cv_image = self.bridge.imgmsg_to_cv2(cropped_image, desired_encoding='passthrough')
            plt.imshow(cv_image)
            ax.xaxis.set_ticklabels([])
            ax.yaxis.set_ticklabels([])
            ax.xaxis.set_ticks([])
            ax.yaxis.set_ticks([])
            ax.set_xlabel(str(image_idx), color='green')
  
        plt.draw()

        key = plt.waitforbuttonpress(1)
        if not plt.fignum_exists(1):
            print('Terminating')
            exit(0)


def main():

    ###########################################
    # Initialization                          #
    ###########################################
    parser = argparse.ArgumentParser(description='Data Collector')
    parser.add_argument('-v', '--visualize', action='store_true', help='Visualize extracted images')
    
    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(parser.parse_args(args=arglist))

    rospy.init_node('image_extractor', anonymous=False)

    image = Image()

    rospy.Subscriber("objects_publisher", Object, image.callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if args['visualize']: # Checks if the user wants to visualize the point cloud
            image.draw()
        rate.sleep() # Sleeps to if time < rate
        


if __name__ == "__main__":
    main()