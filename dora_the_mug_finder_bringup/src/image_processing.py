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