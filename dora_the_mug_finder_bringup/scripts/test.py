#!/usr/bin/env python3

import os
import numpy as np
import string
from scipy.spatial.transform import Rotation as R

import numpy as np



a = np.array([4, 5, 6], dtype=np.float32)
b = np.vstack(a)

print(a)
print(b)

# # input data
# ins = [[1, 1, 2], [2, 3, 0], [3, 2, -2], [-2, 2, 3]]  # <- points
# out = [[0, 2, 1], [1, 2, 2], [-2, -1, 6], [4, 1, -3]] # <- mapped to
# # calculations
# l = len(ins)
# B = np.vstack([np.transpose(ins), np.ones(l)])
# D = 1.0 / np.linalg.det(B)
# entry = lambda r,d: np.linalg.det(np.delete(np.vstack([r, B]), (d+1), axis=0))
# M = [[(-1)**i * D * entry(R, i) for i in range(l)] for R in np.transpose(out)]
# print(np.array(M))



# A, t = np.hsplit(np.array(M), [l-1])
# new = np.zeros((4,4))
# new[0:3,0:3] = A
# t = np.transpose(t)[0]
# new[0:3,3] = t
# new[3,3] = 1
# print("new:\n", new)

# print(np.linalg.inv(new))
# # output

# print("Affine transformation matrix:\n", A)
# print("Affine transformation translation vector:\n", t)



# files_path=f'{os.environ["DORA"]}'

# # Scene dataset paths
# filename = (files_path + '/rgbd-scenes-v2/pc/031.pose')


# with open(filename, "r") as pose:
#         file_pose = pose.readlines()
#         #print(file_pose)

#         vector_pose_str = file_pose[1]
#         #print(vector_pose_str)

#         vector_pose_array = vector_pose_str.split(' ')
#         #print(vector_pose_array) 
#         #print(vector_pose_array[0], vector_pose_array[1], vector_pose_array[2], vector_pose_array[3])
#         r = R.from_quat([vector_pose_array[0], vector_pose_array[1], vector_pose_array[2], vector_pose_array[3]])
#         rotation_matrix = r.as_matrix()
#         #print(rotation_matrix)
#         #rotation_matrix = np.array(rotation_matrix)
#         #print(type(rotation_matrix))
#         a = np.array([vector_pose_array[0],0,0])
#         print(a)
#         print(type(a))
