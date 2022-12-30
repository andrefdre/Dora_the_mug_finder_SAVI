#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, December 2022.
# --------------------------------------------------

import cv2
import open3d as o3d
from copy import deepcopy
import numpy as np
from matplotlib import cm
from more_itertools import locate
import os
import matplotlib.pyplot as plt
import glob
import math
from time import sleep

from point_cloud_processing_ap1 import PointCloudProcessing
	
#! NOT WORKING
def main():

    #Initial view parameters
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

    # ------------------------------------------
    # Initialization
    # ------------------------------------------
    p = PointCloudProcessing() #Calls the class 
    # General Path
    files_path=f'{os.environ["DORA"]}'
    # Scene dataset paths
    scene_paths = glob.glob(files_path + '/rgbd-scenes-v2/pc/*.ply')

    print(files_path + '/rgbd-scenes-v2/*.ply')

    
  
    for scene_path in scene_paths:
        p.loadPointCloud(scene_path) #Gives the filename to class
        
        # ------------------------------------------
        # Execution
        # ------------------------------------------    
        p.preProcess(voxel_size=0.01) # value 0.001#PreProcessing

        # #?Center the origin in the table
        # p.transform(-110, 0, 0, 0, 0, 0) #Rotate in x
        # p.transform(0, 0, 52, 0, 0, 0) #Rotate in z
        # p.transform(0, 0, 0, 1.1, -0.8, 0.37) #Translation in xyz
        
        angle_tolerance = 10
        vx, vy, vz = 0,0,1
        norm_b = math.sqrt(vx**2 + vy**2 + vz**2)
        vertical_idxs = []
        for idx, normal in enumerate(p.pcd.normals):
            nx, ny, nz = normal
            ab = nx*vx + ny*vy + nz*vz
            norm_a = math.sqrt(nx**2 + ny**2 + nz**2)
            angle = math.acos(ab/(norm_a * norm_b)) * 180/ math.pi

            if angle < angle_tolerance: # this point has a "vertical normal"
                vertical_idxs.append(idx)

        vertical_cloud = p.pcd.select_by_index(vertical_idxs)
        non_vertical_cloud = p.pcd.select_by_index(vertical_idxs, invert=True)

        vertical_cloud.paint_uniform_color([0.5, 0, 1]) # paints the plane in X color

        entities = [vertical_cloud, non_vertical_cloud]
        frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=3.0, origin=np.array([0., 0., 0.]))
        entities.append(frame)

        #Create a frame with the orthogonal directions
        #frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
        print(scene_path)
        

        o3d.visualization.draw_geometries(entities, 
                                zoom=view['trajectory'][0]['zoom'],
                                front=view['trajectory'][0]['front'],
                                lookat=view['trajectory'][0]['lookat'],
                                up=view['trajectory'][0]['up'])

     
        #?Creates a bbox of points to keep
        p.crop(-0.9, -0.9, 0, 0.9, 0.9, 0.3)
        #Order: (x,y,z) min and (x,y,z) max
        


        outliers = p.findPlane() #Find the table plane
        p.inliers.paint_uniform_color([0,1,0]) #Paints the plane in green

        #?Now we isolate the objects (cluster the points for each one)
        cluster_idxs = list(outliers.cluster_dbscan(eps=0.035, min_points=60, print_progress=True))
        #Cluster returns points associated to an ID and gives the same ID for points in the defined range
        #If eps small, no object is going to be detected
        
        object_idxs = list(set(cluster_idxs))
        object_idxs.remove(-1) #Removes -1 cluster ID (-1 are the points not clustered)

        number_of_objects = len(object_idxs)
        
        #Color each cluster with a different color using colormap
        # colormap = cm.Pastel1(list(range(0,number_of_objects)))

        #Create the objects list
        objects = []
        #Here we find the points for each object and reunite them 
        for object_idx in object_idxs:

            #Locate the points, comparing each cluster id to the object id, grouping them and converts the result to list because it's easier
            object_point_idxs = list(locate(cluster_idxs, lambda x: x == object_idx))
            object_points = outliers.select_by_index(object_point_idxs)
            
            #Create a dictionary to define the objects
            d = {}
            d['idx'] = str(object_idx)
            d['points'] = object_points
            # d['color'] = colormap[object_idx, 0:3]
            # d['points'].paint_uniform_color(d['color']) #Paints the point in the defined color
            d['center'] = d['points'].get_center()
            d['bbox_obj'] = d['points'].get_axis_aligned_bounding_box()
            
            #Properties of objects (length, width, height)
            bbox_max = d['points'].get_max_bound()
            bbox_min = d['points'].get_min_bound()
            d['length'] = abs(bbox_max[0])-abs(bbox_min[0]) #axis x
            d['width'] = abs(bbox_max[1])-abs(bbox_min[1]) #axis y
            d['height'] = abs(bbox_max[2])-abs(bbox_min[2]) #axis z
                    
            objects.append(d) #Add the dict of this object to the list
        
        # ------------------------------------------
        # Visualization
        # ------------------------------------------
        
        #Create a list of entities to draw
        entities = []

        vis = o3d.visualization.Visualizer()
        vis.create_window()

        #Create a frame with the orthogonal directions
        #frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
        #entities.append(frame)
        #vis.add_geometry(frame)
        
        # Draw bbox
        bbox_to_draw = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(p.bbox)
        entities.append(bbox_to_draw)
        

        # #Draw objects
        # for object in objects:
        #         entities.append(object['points'])

        #Show only object idx = 2
        for object_idx, object in enumerate(objects):
            #if object_idx == 2:    
                bbox_to_draw = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(objects[object_idx]['bbox_obj'])
                #entities.append(bbox_to_draw)
                entities.append(object['points'])
                #vis.add_geometry(bbox_to_draw)
                vis.add_geometry(object['points'])
                
        
        #Draws entities and show PointCloud in the defined view

        vis.run()
        image = vis#.capture_screen_float_buffer()
        plt.imshow(np.asarray(image))
        plt.show()

        # o3d.visualization.draw_geometries(entities, 
        #                                 zoom=view['trajectory'][0]['zoom'],
        #                                 front=view['trajectory'][0]['front'],
        #                                 lookat=view['trajectory'][0]['lookat'],
        #                                 up=view['trajectory'][0]['up'])

        
        # ------------------------------------------
        # Termination
        # ------------------------------------------
        if cv2.waitKey(0) == ord('q'):
            exit()


if __name__ == "__main__":
    main()