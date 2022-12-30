#!/usr/bin/env python3

import csv
import pickle
from copy import deepcopy
from random import randint
from turtle import color

import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import math
import os

#! NOT WORKING
view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 0.90000000000000002, 0.90000000000000002, 0.5 ],
			"boundingbox_min" : [ -0.90000000000000002, -0.90000000000000002, -0.10000000000000003 ],
			"field_of_view" : 60.0,
			"front" : [ 0.57428340315845261, 0.72415504342386261, 0.38183510307530683 ],
			"lookat" : [ -0.23064077816298553, -0.2045093977126011, 0.17408966530741635 ],
			"up" : [ -0.36479928025136604, -0.19118507801017759, 0.91124626258455921 ],
			"zoom" : 0.79999999999999893
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}



class PlaneDetection():
    def __init__(self, point_cloud):

        self.point_cloud = point_cloud

    def colorizeInliers(self, r,g,b):
        self.inlier_cloud.paint_uniform_color([r,g,b]) # paints the plane in red

    def segment(self, distance_threshold=0.03, ransac_n=5, num_iterations=50):

        print('Starting plane detection')
        self.plane_model, inlier_idxs = self.point_cloud.segment_plane(distance_threshold=distance_threshold, 
                                                    ransac_n=ransac_n,
                                                    num_iterations=num_iterations)
        [self.a, self.b, self.c, self.d] = self.plane_model

        self.inlier_cloud = self.point_cloud.select_by_index(inlier_idxs)
        self.outlier_cloud = self.point_cloud.select_by_index(inlier_idxs, invert=True)

    def __str__(self):
        text = 'Segmented plane from pc with ' + str(len(self.point_cloud.points)) + ' with ' + str(len(self.inlier_cloud.points)) + ' inliers. '
        text += '\nPlane: ' + str(self.a) +  ' x + ' + str(self.b) + ' y + ' + str(self.c) + ' z + ' + str(self.d) + ' = 0' 
        return text

def vector_angle(u, v):
    return np.arccos(np.dot(u,v) / (np.linalg.norm(u)* np.linalg.norm(v)))        


def main():
    
    # ------------------------------------------
    # Initialization
    # ------------------------------------------

    files_path=f'{os.environ["DORA"]}'
    # Scene dataset paths
    filename = files_path + '/rgbd-scenes-v2/pc/01.ply'
    
    os.system('pcl_ply2pcd ' + filename + ' pcd_point_cloud.pcd')
    point_cloud_original = o3d.io.read_point_cloud('pcd_point_cloud.pcd')
     
    # ------------------------------------------
    # Execution
    # ------------------------------------------
    point_cloud = deepcopy(point_cloud_original) 
    
    plane = PlaneDetection(point_cloud) # create a new plane instance
    plane.segment() # new point cloud are the outliers of this plane detection
    
    # Get the plane equation of the floor → ax+by+cz+d = 0
    
    # Translate plane to coordinate center
    #point_cloud.translate((0,-plane.d/plane.c,0))

    # Calculate rotation angle between plane normal & z-axis
    plane_normal = tuple(plane.plane_model[:3])
    z_axis = (0,0,1)
    rotation_angle = vector_angle(plane_normal, z_axis)

    # Calculate rotation axis
    plane_normal_length = math.sqrt(plane.a**2 + plane.b**2 + plane.c**2)
    u1 = plane.b / plane_normal_length
    u2 = -plane.a / plane_normal_length
    rotation_axis = (u1, u2, 0)

    # Generate axis-angle representation
    optimization_factor = 1.4
    axis_angle = tuple([x * rotation_angle * optimization_factor for x in rotation_axis])

    # Rotate point cloud
    R = point_cloud.get_rotation_matrix_from_axis_angle(axis_angle)
    point_cloud.rotate(R, center=(0,0,0))

    # #plane2 = PlaneDetection(plane.outlier_cloud)
    # #plane2.segment()
    
    color = [1,0,0]
    plane.colorizeInliers(r=color[0], g=color[1], b=color[2])
    #plane2.colorizeInliers(r=color[0], g=color[1], b=color[2])

    # ------------------------------------------
    # Visualization
    # ------------------------------------------

    #entities = [plane2.inlier_cloud, plane2.outlier_cloud, plane.inlier_cloud]
    #plane_points = plane2.outlier_cloud + plane.inlier_cloud
    #entities = [plane.inlier_cloud, plane.outlier_cloud]
    frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
    entities = [point_cloud]
    entities.append(frame)
    entities.append(plane.inlier_cloud)
    o3d.visualization.draw_geometries(entities,
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

    #o3d.io.write_point_cloud(files_path + '/rgbd-scenes-v2/pc/01_without_ground.ply', plane_points, write_ascii=False, compressed=False, print_progress=False)

   

if __name__ == "__main__":
    main()