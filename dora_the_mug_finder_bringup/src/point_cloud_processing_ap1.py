#!/usr/bin/env python3
# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# Fábio Sousa.
# SAVI, December 2022.
# --------------------------------------------------

import open3d as o3d
from copy import deepcopy
import numpy as np
import math

class PointCloudProcessing():
    def __init__(self):
        super().init()
        pass

    def loadPointCloud(self, filename): #Reads file
        print("Load a point cloud from " + filename)
        self.pcd = o3d.io.read_point_cloud(filename)
        self.original = deepcopy(self.pcd) #Make a backup of the original point cloud

    def preProcess(self,voxel_size=0.02): #PreProcessing of Point Cloud data
        #Downsampling using voxel grid filter
        self.pcd = self.pcd.voxel_down_sample(voxel_size=voxel_size) 
        print('Downsampling reduced point cloud from  ' + str(len(self.original.points)) + ' to ' + str(len(self.pcd.points))+  ' points')

        #Estimate normals
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
        #!Is this a good orientation???
        self.pcd.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., 1.]))
        
    def transform(self, r,p,y,tx,ty,tz): #Change the origin
            #Convert from rad to deg
            r = math.pi * r / 180.0
            p = math.pi * p / 180.0
            y = math.pi * y / 180.0

            #First rotate
            rotation = self.pcd.get_rotation_matrix_from_xyz((r,p,y))
            self.pcd.rotate(rotation, center=(0, 0, 0))

            #Then translate
            self.pcd = self.pcd.translate((tx,ty,tz))

    def crop(self, min_x, min_y, min_z, max_x, max_y, max_z):
        #First create a point cloud with the vertices of the desired bounding box (it has 8 vertices in 3 dimensions)
        np_points = np.ndarray((8,3),dtype=float)

        #The bbox has it's center on the origin, therefore each vertice will be a combination of x, y and z max and min values
        np_points[0,:] = [min_x, min_y, min_z]
        np_points[1,:] = [max_x, min_y, min_z]
        np_points[2,:] = [max_x, max_y, min_z]
        np_points[3,:] = [min_x, max_y, min_z]

        np_points[4,:] = [min_x, min_y, max_z]
        np_points[5,:] = [max_x, min_y, max_z]
        np_points[6,:] = [max_x, max_y, max_z]
        np_points[7,:] = [min_x, max_y, max_z]

        #Convert from numpy to Open3D
        bbox_points = o3d.utility.Vector3dVector(np_points) 

        #Create Axis Aligned Bounding Box from points
        self.bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(bbox_points)
        self.bbox.color = (1, 0, 0) #Colors bbox lines in red

        #Crops the point cloud inside the bbox and keeps it
        self.pcd = self.pcd.crop(self.bbox)

    def findPlane(self, distance_threshold=0.01, ransac_n=3, num_iterations=100):

        print('Segmenting plane from point cloud with ' + str(len(self.pcd.points)) + ' points')
        #Ransac plane
        plane_model, inlier_idxs = self.pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
        #Gets plane equation
        self.a, self.b, self.c, self.d = plane_model
        #Defines inliers as the points within the plane
        self.inliers = self.pcd.select_by_index(inlier_idxs)
        #Outlier is everything else (so that we get the original full point cloud)
        outlier_cloud = self.pcd.select_by_index(inlier_idxs, invert=True)
        return outlier_cloud
