# General Imports 
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


def get_matrix_inv(filename,img_number):
    
        with open(filename, "r") as pose:
            file_pose = pose.readlines()
            vector_pose_str = file_pose[int(img_number)]

            vector_pose_array = vector_pose_str.split(' ')
            
            # rotation matrix
            r = R.from_quat([vector_pose_array[1], vector_pose_array[2], vector_pose_array[3], vector_pose_array[0]])
            rot_matrix = r.as_matrix()
            
            # translation matrix
            trans_matrix = np.array([vector_pose_array[4], vector_pose_array[5], vector_pose_array[6]], dtype=np.float32)

            # homogeneous transformation matrix (4,4) 
            l = 4
            matrix = np.zeros((l,l))
            matrix[0:3,0:3] = rot_matrix
            matrix[0:3,3] = trans_matrix
            matrix[3,3] = 1

            # inverse matrices
            matrix_inv = np.linalg.inv(matrix)
            
            return matrix_inv


def get_iou(bb1_2d,bb2_2d):
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
        #iou = intersection_area / float(bb1_area + bb2_area - intersection_area)
        iou = intersection_area / float(bb1_area)

        return iou


def get_color(image,thickness = 5):
    
    # get the center of the image
    h,w,_ = image.shape 
    image_small = image[round(h/2-thickness):round(h/2+thickness),round(w/2-thickness):round(w/2+thickness)]
    
    # get the mean r,g,b of the image
    r,g,b = cv2.split(image_small)
    mean_r = np.mean(r)
    mean_g = np.mean(g)
    mean_b = np.mean(b)

    # convert to hsv
    hsv_value= np.uint8([[[mean_r,mean_g,mean_b ]]])
    hsv = cv2.cvtColor(hsv_value, cv2.COLOR_RGB2HSV)
    # get the hsv value
    H,S,V = cv2.split(hsv)
 
    # get the color
    if H < 10 and S > 90:
        color = 'red'
    elif H < 15 and S > 90:
        color = 'orange'
    elif H < 30 and S > 90:
        color = 'yellow'
    elif H < 75 and S > 90:
        color = 'green'
    elif H < 125 and S > 90:
        color = 'blue'
    elif H < 155 and S > 90:
        color = 'purple'
    elif H < 180 and S > 90:
        color = 'red'
    elif S <= 90 and V < 120:
        color = 'black'
    else:
        color = 'white'

    return color