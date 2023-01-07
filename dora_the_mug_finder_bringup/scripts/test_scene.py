#!/usr/bin/env python3

from copy import deepcopy

import open3d as o3d
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
import os
from table_detection import PlaneDetection, PlaneTable, Table, Transform
from object_detection import Object_detection
from more_itertools import locate
from math import acos, sqrt, pi



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
    filename = files_path + '/rgbd-scenes-v2/pc/01.ply'
    
    os.system('pcl_ply2pcd ' + filename + ' pcd_point_cloud.pcd')
    point_cloud_original = o3d.io.read_point_cloud('pcd_point_cloud.pcd')
     
    # ------------------------------------------
    # Execution
    # ------------------------------------------

    #? find two planes: from the table and another
    point_cloud = deepcopy(point_cloud_original)
    point_cloud_complet = deepcopy(point_cloud_original) 
    number_of_planes = 2
    colormap = cm.Set1(list(range(0,number_of_planes)))
    planes = []
    inlieres_idxs = []
    while True: # run consecutive plane detections

        plane = PlaneDetection(point_cloud) # create a new plane instance
        point_cloud, inlier_idxs = plane.segment() # new point cloud are the outliers of this plane detection
 
        # colorization using a colormap
        idx_color = len(planes)
        color = colormap[idx_color, 0:3]
        plane.colorizeInliers(r=color[0], g=color[1], b=color[2])

        planes.append(plane)
        inlieres_idxs.append(inlier_idxs)

        if len(planes) >= number_of_planes: # stop detection planes
            break
    
    
    #? find table plane
    plane_table,inlier_idxs = PlaneTable(planes,inlieres_idxs).planetable()


    #? find table
    t = Table()
    table = t.voxel_dow_sample(plane_table.inlier_cloud)
    cluster_idxs, object_idxs = t.cluster(table)
    table = t.table(cluster_idxs, object_idxs, table)

    #? frame alignment
    frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
    tx, ty, tz = table.get_center()
    x, y, z = plane_table.a, plane_table.b, plane_table.c
    
    point_cloud_complet = Transform(0,0,0,-tx,-ty,-tz).translate(point_cloud_complet)
    point_cloud = Transform(0,0,0,-tx,-ty,-tz).translate(point_cloud)
    table = Transform(0,0,0,-tx,-ty,-tz).translate(table)
    
    point_cloud_complet = Transform(-x,-y,-z,0,0,0).rotate(point_cloud_complet)
    point_cloud = Transform(-x,-y,-z,0,0,0).rotate(point_cloud)
    table = Transform(-x,-y,-z,0,0,0).rotate(table)
  
    #? bbox: table + objects
    bbox = Object_detection().bbox(table)
    
    
    #? objects + noise
    point_cloud_complet = point_cloud_complet.select_by_index(inlier_idxs, invert=True)
    #Crops the point cloud inside the bbox and keeps it
    point_cloud = point_cloud_complet.crop(bbox)

    # #? objects
    # cluster_idxs = list(point_cloud.cluster_dbscan(eps=0.025, min_points=60, print_progress=True))
    
    # object_idxs = list(set(cluster_idxs))
    # object_idxs.remove(-1) #Removes -1 cluster ID (-1 are the points not clustered)

    # number_of_objects = len(object_idxs)
    
    # #Color each cluster with a different color using colormap
    # # colormap = cm.Pastel1(list(range(0,number_of_objects)))

    # #Create the objects list
    # objects = []
    # #Here we find the points for each object and reunite them 
    # for object_idx in object_idxs:

    #     #Locate the points, comparing each cluster id to the object id, grouping them and converts the result to list because it's easier
    #     object_point_idxs = list(locate(cluster_idxs, lambda x: x == object_idx))
    #     object_points = point_cloud.select_by_index(object_point_idxs)
        
    #     #Create a dictionary to define the objects
    #     d = {}
    #     d['idx'] = str(object_idx)
    #     d['points'] = object_points
                
    #     objects.append(d) #Add the dict of this object to the list
    

    
    # ------------------------------------------
    # Visualization
    # ------------------------------------------
    entities = []
    # for object in objects:
    #         entities.append(object['points'])
    entities.append(point_cloud)
    entities.append(frame)
    #entities.append(table)
    entities.append(bbox)
    
    
 
    o3d.visualization.draw_geometries(entities,
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

   

if __name__ == "__main__":
    main()