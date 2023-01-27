#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------

# General Imports 
from scipy.spatial.transform import Rotation as R
from colorama import Fore, Style
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import argparse
import rospy
import glob
import os
import cv2
import sys
import yaml

from sensor_msgs.msg import Image

# Own package imports
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
        filenames = sorted(glob.glob(files_path + f'/rgbd-scenes-v2/imgs/{data.scene.data}/*-color.png'))
        
        if data.scene.data == 'kinect':
            # Camera parameters
            with open(f'{files_path}/rosbag/intrinsic.yaml', "r") as yamlfile:
                camera_data = yaml.load(yamlfile, Loader=yaml.FullLoader)
            center = [camera_data['camera_matrix']['data'][2] , camera_data['camera_matrix']['data'][5]]
            focal_length = camera_data['camera_matrix']['data'][0]

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
        
            # Line thickness of 5 px
            thickness = 5

            self.cropped_images = Images()
            image = cv2.cvtColor(self.kinect_image,cv2.COLOR_RGB2BGR)
            for idx,point_2d in enumerate(points_2d):
                    #image[point_2d[0][1]-2:point_2d[0][1]+2,point_2d[0][0]-2:point_2d[0][0]+2]=color
                    #image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
                    cropped_image = image[bbox_2d[idx][1][0][1]:bbox_2d[idx][0][0][1],bbox_2d[idx][0][0][0]:bbox_2d[idx][1][0][0]]
                    print(cropped_image)
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


            for filename in filenames:

                image_number = filename.split('/')[-1].split('-')[0]
              
                camera_matrix = np.array([[focal_length, 0,            center[0]],
                                        [0,            focal_length, center[1]],
                                        [0,            0,            1]])

                ########################################
                # Points & Bbox 3D                     #
                ########################################
                points = np.array([[center.x,center.y,center.z] for center in data.center],dtype = np.float64)
                bbox_3d =np.array( [[[data.corners[idx].x,data.corners[idx+1].y+0.05,data.corners[idx].z],[data.corners[idx+1].x,data.corners[idx].y,data.corners[idx].z]] for idx in range(0,len(data.corners),2)] ,dtype=np.float64)                    
                
                # Apply matriz inverse: points, bbox_3d 
                self.get_matrix_inv(filename_pose,image_number)

                for idx, _ in enumerate(points):
                    point = np.ones((1,4))
                    point[:,0:3] = points[idx]
                    point = np.transpose(point)
                    point = np.dot(self.matrix_inv,point)
                    points[idx,:]=np.transpose(point[0:3,:]) 

                for idx, _ in enumerate(bbox_3d): 
                    for n in range(0,2):
                        bbox = np.ones((1,4))
                        bbox[:,0:3] = bbox_3d[idx][n]
                        bbox = np.transpose(bbox)
                        bbox = np.dot(self.matrix_inv,bbox)
                        bbox_3d[idx][n]=np.transpose(bbox[0:3,:]) 

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
                bbox_2d = abs(np.round(bbox_2d).astype(int))

                # Blue color in BGR
                color = (0, 251, 255)
            
                # Line thickness of 5 px
                thickness = 5

                self.cropped_images = Images()
            
                # create image
                image = cv2.imread(filename)
                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

                ########################################
                #                                      #
                ########################################
                
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
                try:
                    iou_overlap = False
                    for idx1 , bbox in enumerate(bbox_2d,start=1):
                        bb1_2d = bbox
                        for idx2 , bbox in enumerate(bbox_2d,start=1):
                            bb2_2d = bbox
                            self.get_iou(bb1_2d,bb2_2d)
                            if idx1 != idx2:
                                print('img:' + str(idx1) + ' with img:' + str(idx2) + ' iou:'+ str(self.iou_small))
                                if self.iou_small > 0.6:
                                    iou_overlap = True
                                if self.iou_small == 1:
                                    iou_overlap = True
                    if iou_overlap:
                        continue
                except:
                    continue
                        
                # cropped images
                for idx,point_2d in enumerate(points_2d):
                    image[point_2d[0][1]-thickness:point_2d[0][1]+thickness,point_2d[0][0]-thickness:point_2d[0][0]+thickness]=color
                    #image = cv2.rectangle(image, bbox_2d[idx][0][0], bbox_2d[idx][1][0], color, thickness)
                    cropped_image = image[bbox_2d[idx][1][0][1]:bbox_2d[idx][0][0][1],bbox_2d[idx][0][0][0]:bbox_2d[idx][1][0][0]]
                    if cropped_image.shape[0] == 0 and cropped_image.shape[1] == 0:
                        print(f'{Fore.RED}Skipping Image due to inappropriate width/height. {Style.RESET_ALL}')
                        continue                       
                    self.cropped_images.images.append(self.bridge.cv2_to_imgmsg(cropped_image, "passthrough"))
                
                
                break
                
        self.pub.publish(self.cropped_images)


    def get_matrix_inv(self,filename,img_number):
        
        with open(filename, "r") as pose:
            file_pose = pose.readlines()
            vector_pose_str = file_pose[int(img_number)]

            vector_pose_array = vector_pose_str.split(' ')
            
            # rotation matrix
            r = R.from_quat([vector_pose_array[1], vector_pose_array[2], vector_pose_array[3], vector_pose_array[0]])
            self.rot_matrix = r.as_matrix()
            
            # translation matrix
            self.trans_matrix = np.array([vector_pose_array[4], vector_pose_array[5], vector_pose_array[6]], dtype=np.float32)

            # homogeneous transformation matrix (4,4) 
            l = 4
            self.matrix = np.zeros((l,l))
            self.matrix[0:3,0:3] = self.rot_matrix
            self.matrix[0:3,3] = self.trans_matrix
            self.matrix[3,3] = 1

            # inverse matrices
            self.matrix_inv = np.linalg.inv(self.matrix)


    def get_iou(self,bb1_2d,bb2_2d):
        """
        Calculate the Intersection over Union (IoU) of two bounding boxes.

        bb1 : dict
            Keys: {'x1', 'x2', 'y1', 'y2'}
            The (x1, y1) position is at the top left corner,
            the (x2, y2) position is at the bottom right corner
        bb2 : dict
            Keys: {'x1', 'x2', 'y1', 'y2'}
            The (x, y) position is at the top left corner,
            the (x2, y2) position is at the bottom right corner
        """

        bb1 = {'x1': bb1_2d[0][0][0] , 'x2': bb1_2d[1][0][0] , 'y1': bb1_2d[1][0][1] , 'y2': bb1_2d[0][0][1]}
        bb2 = {'x1': bb2_2d[0][0][0] , 'x2': bb2_2d[1][0][0] , 'y1': bb2_2d[1][0][1] , 'y2': bb2_2d[0][0][1]}

        assert bb1['x1'] < bb1['x2']
        assert bb1['y1'] < bb1['y2']
        assert bb2['x1'] < bb2['x2']
        assert bb2['y1'] < bb2['y2']
        
        # determine the coordinates of the intersection rectangle
        x_left = max(bb1['x1'], bb2['x1'])
        y_top = max(bb1['y1'], bb2['y1'])
        x_right = min(bb1['x2'], bb2['x2'])
        y_bottom = min(bb1['y2'], bb2['y2'])
        
        # The intersection of two axis-aligned bounding boxes is always an
        # axis-aligned bounding box
        intersection_area = abs(max((x_right - x_left),0) * max((y_bottom - y_top),0))
        
        # compute the area of both AABBs
        bb1_area = (bb1['x2'] - bb1['x1']) * (bb1['y2'] - bb1['y1'])
        bb2_area = (bb2['x2'] - bb2['x1']) * (bb2['y2'] - bb2['y1'])

        # compute the intersection over union 
        self.iou_int = intersection_area / float(bb1_area + bb2_area - intersection_area)
        self.iou_small = intersection_area / float(bb1_area)
        

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