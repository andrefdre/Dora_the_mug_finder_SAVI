#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------

# General Imports 
from colorama import Fore, Style
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import rospy
import glob
import os
import cv2
import yaml
import argparse
import sys

from sensor_msgs.msg import Image

# Own package imports
from dora_the_mug_finder_msg.msg import Object , Images
from dora_the_mug_finder_bringup.src.image_processing import get_matrix_inv, get_iou, get_color

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
        filenames = sorted(glob.glob(files_path + f'/rgbd-scenes-v2/imgs/{data.scene.data}/*-color.png'))
        
        if data.scene.data == 'kinect':
            # Camera parameters
            with open(f'{files_path}/rosbag/intrinsic.yaml', "r") as yamlfile:
                camera_data = yaml.load(yamlfile, Loader=yaml.FullLoader)
            center = [camera_data['camera_matrix']['data'][2] , camera_data['camera_matrix']['data'][5]]
            focal_length = camera_data['camera_matrix']['data'][0]

            camera_matrix = np.array([[focal_length, 0,            center[0]],
                                    [0,            focal_length,   center[1]],
                                    [0,            0,              1]])

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
        
            # Line thickness of 5 px
            thickness = 5

            self.cropped_images = Images()
            image = cv2.cvtColor(self.kinect_image,cv2.COLOR_RGB2BGR)
            for idx_objects,point_2d in enumerate(points_2d):
                    #image[point_2d[0][1]-2:point_2d[0][1]+2,point_2d[0][0]-2:point_2d[0][0]+2]=color
                    #image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
                    width = bbox_2d[idx_objects][1][0][0] - bbox_2d[idx_objects][0][0][0]
                    height = bbox_2d[idx_objects][0][0][1] - bbox_2d[idx_objects][1][0][1]
                    cropped_image = image[round(point_2d[0][1]-height/2):round(point_2d[0][1]+height/2),round(point_2d[0][0]-width/2):round(point_2d[0][0]+width/2)]
                    if cropped_image.shape[0] == 0 or cropped_image.shape[1] == 0:
                        print(f'{Fore.RED}Skipping Image due to inappropriate width/height. {Style.RESET_ALL}')
                        continue 
                    self.cropped_images.images.append(self.bridge.cv2_to_imgmsg(cropped_image, "passthrough"))
        
        else:
            
            # Pose dataset paths
            scene_number = data.scene.data.split('_')  
            filename_pose = (files_path + f'/rgbd-scenes-v2/pc/{scene_number[1]}.pose')

            # Camera parameters
            center = [320 , 240]
            focal_length = 570.3

            camera_matrix = np.array([[focal_length, 0,            center[0]],
                                    [0,            focal_length, center[1]],
                                    [0,            0,            1]])
            
            # initialization variables
            images_dic = []
            idx_images = list(range(len(data.center)))
            idx_images_cropped = list(range(len(data.center)))
            
            for filename in filenames:

                ########################################
                # Points & Bbox 3D                     #
                ########################################
                
                # Get the 3D points and bbox 3D
                points = np.array([[center.x,center.y,center.z] for center in data.center],dtype = np.float64)
                bbox_3d =np.array( [[[data.corners[idx].x,data.corners[idx+1].y+0.05,data.corners[idx].z],[data.corners[idx+1].x,data.corners[idx].y,data.corners[idx].z]] for idx in range(0,len(data.corners),2)] ,dtype=np.float64)                    
                
                # Get the image number
                image_number = filename.split('/')[-1].split('-')[0]
                
                # Apply matriz inverse: points, bbox_3d 
                matrix_inv = get_matrix_inv(filename_pose,image_number)
    
                for idx_objects, _ in enumerate(points):
                    point = np.ones((1,4))
                    point[:,0:3] = points[idx_objects]
                    point = np.transpose(point)
                    point = np.dot(matrix_inv,point)
                    points[idx_objects,:]=np.transpose(point[0:3,:]) 

                for idx_objects, _ in enumerate(bbox_3d): 
                    for n in range(0,2):
                        bbox = np.ones((1,4))
                        bbox[:,0:3] = bbox_3d[idx_objects][n]
                        bbox = np.transpose(bbox)
                        bbox = np.dot(matrix_inv,bbox)
                        bbox_3d[idx_objects][n]=np.transpose(bbox[0:3,:]) 

                ########################################
                # Points & Bbox 2D                     #
                ########################################
                
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
            
                # Line thickness of 5 px
                thickness = 5

                # initialize the cropped images
                self.cropped_images = Images()
            
                # create image
                image = cv2.imread(filename)
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

                ###############################################
                # Check the conditions for cropping the image #                                    #
                ###############################################
                
                # check the bbox inside the image
                h, w, _ = image.shape
                ymaxs = [ymax[0][0][1] for ymax in bbox_2d]
                ymins = [ymin[1][0][1] for ymin in bbox_2d]
                xmaxs = [xmax[1][0][0] for xmax in bbox_2d]
                xmins = [xmin[0][0][0] for xmin in bbox_2d]
                x_pts = [xmin_pts[0][0] for xmin_pts in points_2d]
                y_pts = [ymin_pts[0][1] for ymin_pts in points_2d]         
                if min(ymins) < 0 or min(xmins) < 0 or max(xmaxs) > w or max(ymaxs) > h or min(x_pts) <= 0 or min(y_pts) <= 0:
                    continue
                
                # check IOU bbox
                idx_images_overlap = []
                for idx1 in idx_images_cropped:
                    bb1x_2d = bbox_2d[idx1]
                    
                    for idx2 in idx_images:
                        bbx2_2d = bbox_2d[idx2]
                        iou_bbx1 = get_iou(bb1x_2d,bbx2_2d)
                        
                        if idx1 != idx2 and iou_bbx1 > 0.05 and not idx1 in idx_images_overlap:                     
                            idx_images_overlap.append(idx1)

                for idx in idx_images_overlap:
                    idx_images_cropped.remove(idx)
           

                ###############################################
                # Cropped the image                           #                                    
                ###############################################
                
                for idx_object in idx_images_cropped:
                    #image[points_2d[idx_object][0][1]-thickness:points_2d[idx_object][0][1]+thickness,points_2d[idx_object][0][0]-thickness:points_2d[idx_object][0][0]+thickness]=color
                    width = bbox_2d[idx_object][1][0][0] - bbox_2d[idx_object][0][0][0]
                    height = bbox_2d[idx_object][0][0][1] - bbox_2d[idx_object][1][0][1]
                    cropped_image = image[round(points_2d[idx_object][0][1]-height/2):round(points_2d[idx_object][0][1]+height/2),round(points_2d[idx_object][0][0]-width/2):round(points_2d[idx_object][0][0]+width/2)]
                    
                    # dictionary with idx_object and image cropped
                    img_cropped = {}
                    img_cropped['idx_object'] = idx_object
                    img_cropped['image'] = cropped_image
                    img_cropped['color'] = get_color(cropped_image,thickness=5)

                    images_dic.append(img_cropped)
                    
                idx_images_cropped = idx_images_overlap

                if len(idx_images_overlap) == 0:
                    break
        
        # sort list of the dicionary
        images_dic = sorted(images_dic, key=lambda d: d['idx_object'])
        
        # publish the cropped images
        for images in images_dic:
            self.cropped_images.images.append(self.bridge.cv2_to_imgmsg(images['image'], "passthrough"))
            
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