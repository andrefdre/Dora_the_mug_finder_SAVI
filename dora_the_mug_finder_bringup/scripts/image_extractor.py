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
import yaml


import rospy
from dora_the_mug_finder_msg.msg import Object , Images


class ROSHandler:

    def __init__(self):
        self.figure = plt.figure("Cropped Images")
        self.figure.set_size_inches(10,5)
        self.cropped_images = Images()
        self.pub = rospy.Publisher('image_publisher', Images, queue_size=10)
        self.bridge = CvBridge()

    def callback_image(self,data):
        self.kinect_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def callback(self,data):
        files_path=f'{os.environ["DORA"]}'

        # Scene images
        filenames = []
        if data.scene.data == 'kinect':
            # Camera parameters
            with open(f'{files_path}/rosbag/intrinsic.yaml', "r") as yamlfile:
                data = yaml.load(yamlfile, Loader=yaml.FullLoader)
            print(data)
            center = [data['camera_matrix']['data'][2] , data['camera_matrix']['data'][5]]
            focal_length = data['camera_matrix']['data'][0]

            camera_matrix = np.array([[focal_length, 0,            center[0]],
                                    [0,            focal_length, center[1]],
                                    [0,            0,            1]])

        else:
            # Scene dataset paths
            filenames.append (files_path + f'/rgbd-scenes-v2/imgs/{data.scene.data}/00000-color.png')

            # Camera parameters
            center = [320 , 240]
            focal_length = 570.3

            camera_matrix = np.array([[focal_length, 0,            center[0]],
                                    [0,            focal_length, center[1]],
                                    [0,            0,            1]])


        points = np.array([[center.x,center.y,center.z] for center in data.center],dtype = np.float64)
        bbox_3d =np.array( [[[data.corners[idx].x,data.corners[idx+1].y+0.05,data.corners[idx].z],[data.corners[idx+1].x,data.corners[idx].y,data.corners[idx].z]] for idx in range(0,len(data.corners),2)] ,dtype=np.float64)                    
        
        # Project the 3D points to the 2D image plane
        points_2d = cv2.projectPoints(points, np.identity(3), np.zeros(3), camera_matrix, None,)[0]

        bbox_2d = []

        for corners in bbox_3d:
            bbox_2d.append(cv2.projectPoints(corners, np.identity(3), np.zeros(3), camera_matrix, None,)[0])

        # Scale the points to image pixels
        points_2d = np.round(points_2d).astype(int)
        bbox_2d = abs(np.round(bbox_2d).astype(int))

        # Blue color in BGR
        color = (0, 251, 255)
    
        # Line thickness of 2 px
        thickness = 2

        self.cropped_images = Images()
        if data.scene.data == 'kinect':
            image = cv2.cvtColor(self.kinect_image,cv2.COLOR_RGB2BGR)
            for idx,point_2d in enumerate(points_2d):
                    #image[point_2d[0][1]-2:point_2d[0][1]+2,point_2d[0][0]-2:point_2d[0][0]+2]=color
                    #image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
                    cropped_image = image[bbox_2d[idx][1][0][1]:bbox_2d[idx][0][0][1],bbox_2d[idx][0][0][0]:bbox_2d[idx][1][0][0]]
                    height ,width , _ = cropped_image.shape
                    self.cropped_images.images.append(self.bridge.cv2_to_imgmsg(cropped_image, "passthrough"))
        else:
            for filename in filenames:
                image = cv2.imread(filename)
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

                for idx,point_2d in enumerate(points_2d):
                    #image[point_2d[0][1]-2:point_2d[0][1]+2,point_2d[0][0]-2:point_2d[0][0]+2]=color
                    #image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
                    cropped_image = image[bbox_2d[idx][1][0][1]:bbox_2d[idx][0][0][1],bbox_2d[idx][0][0][0]:bbox_2d[idx][1][0][0]]
                    height ,width , _ = cropped_image.shape
                    self.cropped_images.images.append(self.bridge.cv2_to_imgmsg(cropped_image, "passthrough"))

        self.pub.publish(self.cropped_images)

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

    ros_handler = ROSHandler()

    rospy.Subscriber("objects_publisher", Object, ros_handler.callback)
    rospy.Subscriber("/camera/rgb/image_color", Image, ros_handler.callback_image)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if args['visualize']: # Checks if the user wants to visualize the point cloud
            ros_handler.draw()
        rate.sleep() # Sleeps to if time < rate
        


if __name__ == "__main__":
    main()