#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------

import math
from more_itertools import locate
from math import sqrt
from copy import deepcopy
import open3d as o3d
import numpy as np
import glob
import sys

sys.path.append('/home/fabio/catkin_ws/src/Dora_the_mug_finder_SAVI/dora_the_mug_finder_bringup/src')

from dora_the_mug_finder_bringup.src.table_detection import PlaneDetection, PlaneTable, Table, Transform

def keypoints_to_spheres(keypoints):
    spheres = o3d.geometry.TriangleMesh()
    for keypoint in keypoints.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        sphere.translate(keypoint)
        spheres += sphere
    spheres.paint_uniform_color([1.0, 0.75, 0.0])
    return spheres

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

def main():
    
    # ------------------------------------------
    # Initialization
    # ------------------------------------------

    files_path=f'{os.environ["DORA"]}'
    
    # Scene dataset paths
    filenames = []
    filenames.append (files_path + '/rgbd-scenes-v2/pc/03.ply')
    #filenames = glob.glob(files_path + '/rgbd-scenes-v2/pc/*.ply')

    for filename in filenames:
        os.system('pcl_ply2pcd ' + filename + ' pcd_point_cloud.pcd')
        point_cloud_original = o3d.io.read_point_cloud('pcd_point_cloud.pcd')
        
        # ------------------------------------------
        # Execution
        # ------------------------------------------

        #? find two planes: from the table and another        
        point_cloud_twoplanes = deepcopy(point_cloud_original) 
        number_of_planes = 2
        planes = []
        while True: # run consecutive plane detections

            plane = PlaneDetection(point_cloud_twoplanes) # create a new plane instance
            point_cloud_twoplanes = plane.segment() # new point cloud are the outliers of this plane detection
    
            planes.append(plane)

            if len(planes) >= number_of_planes: # stop detection planes
                break

    
        # #? find table plane
        plane_table = PlaneTable(planes) # create a new plane_table instance
        plane_table = plane_table.planetable()


        # properties of the plane_table: .a,.b,.c,.d,
        #                                .inlier_cloud,.inlier_idxs,
        #                                .outlier_cloud
        

        #? definition point cloud without table and point cloud only table
        plane_table.outlier_cloud = plane_table.outlier_cloud.voxel_down_sample(voxel_size=0.005)
        

        #? find table
        t = Table()     # object initialization 
        t.voxel_down_sample(plane_table.inlier_cloud,voxel_size=0.005)
        t.cluster(t.down_sampled_plane)
        t.table(t.cluster_idxs, t.object_idxs, t.down_sampled_plane)


        #? frame alignment
        frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
        tx, ty, tz = t.table.get_center()
        x, y, z = plane_table.a, plane_table.b, plane_table.c
        
        plane_table.outlier_cloud = Transform(0,0,0,-tx,-ty,-tz).translate(plane_table.outlier_cloud)
        t.table = Transform(0,0,0,-tx,-ty,-tz).translate(t.table)
        
        plane_table.outlier_cloud = Transform(-x,-y,-z,0,0,0).rotate(plane_table.outlier_cloud)
        t.table = Transform(-x,-y,-z,0,0,0).rotate(t.table)
    

        #? bbox: table + objects
        t.bbox_table(t.table)


        #? objects + noise
        point_cloud_objects_noise = plane_table.outlier_cloud.crop(t.bbox)
  

        #? objects
        cluster_idxs = list(point_cloud_objects_noise.cluster_dbscan(eps=0.025, min_points=100, print_progress=True))
        object_idxs = list(set(cluster_idxs))
        object_idxs.remove(-1) #Removes -1 cluster ID (-1 are the points not clustered)

        objects = []    #Create the objects list
        threshold_z = 0
        threshold_dist = 0.7
        threshold_width = 0.7
        threshold_length = 0.7
        #Here we find the points for each object and reunite them 
        for object_idx in object_idxs:

            #Locate the points, comparing each cluster id to the object id, grouping them and converts the result to list because it's easier
            object_point_idxs = list(locate(cluster_idxs, lambda x: x == object_idx))
            object_points = point_cloud_objects_noise.select_by_index(object_point_idxs)
            
            #Create a dictionary to define the objects
            d = {}
            d['idx'] = str(object_idx)
            d['points'] = object_points
            
            #Properties of objects (length, width, height)
            bbox_max = d['points'].get_max_bound()
            bbox_min = d['points'].get_min_bound()
            d['x'],d['y'],d['z'] = d['points'].get_center()
            dist = sqrt(d['x']**2 + d['y']**2)
            d['length'] = abs(abs(bbox_max[0])-abs(bbox_min[0])) #axis x
            d['width'] = abs(abs(bbox_max[1])-abs(bbox_min[1])) #axis y
            d['height'] = abs(abs(bbox_max[2])-abs(bbox_min[2])) #axis z

            if d['z'] > threshold_z and dist < threshold_dist and d['width'] < threshold_width and d['length'] < threshold_length:       
                # condition of being object: Z center > 0, be close to the reference, not be too big
                objects.append(d) #Add the dict of this object to the list
        

        # ------------------------------------------
        # Visualization
        # ------------------------------------------

        entities = []
        for object_idx, object in enumerate(objects):
            object['points'] = Transform(-x,y,z,0,0,0).rotate(object['points'],inverse=True)
            object['points'] = Transform(0,0,0,tx,ty,tz).translate(object['points'])
            object['bbox_obj'] = object['points'].get_axis_aligned_bounding_box()
            bbox_to_draw = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(object['bbox_obj'])
            entities.append(object['points'])
            entities.append(bbox_to_draw)
            center = object['points'].get_center()
            sphere =o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            sphere.paint_uniform_color([1.0, 0.75, 0.0])
            sphere.translate(center)
            entities.append(sphere)
            print(center)
            #keypoints = o3d.geometry.keypoint.compute_iss_keypoints(object['points'])
            #entities.append(keypoints_to_spheres(keypoints))

        #entities.append(t.bbox)
        entities.append(frame)
        entities.append(point_cloud_original)

        o3d.visualization.draw_geometries(entities,
                                        zoom=view['trajectory'][0]['zoom'],
                                        front=view['trajectory'][0]['front'],
                                        lookat=view['trajectory'][0]['lookat'],
                                        up=view['trajectory'][0]['up'])

    

if __name__ == "__main__":
    main()