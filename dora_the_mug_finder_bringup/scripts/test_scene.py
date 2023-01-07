#!/usr/bin/env python3

from copy import deepcopy

import open3d as o3d
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
import os
from table_detection import PlaneDetection, PlaneTable, Table
from more_itertools import locate
import random
import math as sqrt


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
    filename = files_path + '/rgbd-scenes-v2/pc/13.ply'
    
    os.system('pcl_ply2pcd ' + filename + ' pcd_point_cloud.pcd')
    point_cloud_original = o3d.io.read_point_cloud('pcd_point_cloud.pcd')
     
    # ------------------------------------------
    # Execution
    # ------------------------------------------

    #? find two planes: from the table and another
    point_cloud = deepcopy(point_cloud_original) 
    number_of_planes = 2
    colormap = cm.Set1(list(range(0,number_of_planes)))
    planes = []
    while True: # run consecutive plane detections

        plane = PlaneDetection(point_cloud) # create a new plane instance
        point_cloud = plane.segment() # new point cloud are the outliers of this plane detection
        print(plane)
 
        # colorization using a colormap
        idx_color = len(planes)
        color = colormap[idx_color, 0:3]
        plane.colorizeInliers(r=color[0], g=color[1], b=color[2])

        planes.append(plane)

        if len(planes) >= number_of_planes: # stop detection planes
            break
    
    #? find table plane
    plane_table = PlaneTable(planes).planetable()

    #? cluster
    t = Table()
    table = t.voxel_dow_sample(plane_table.inlier_cloud)
    cluster_idxs, object_idxs = t.cluster(table)
    table = t.table(cluster_idxs, object_idxs, table)

   
    # ------------------------------------------
    # Visualization
    # ------------------------------------------
    entities = [plane_table.inlier_cloud]
    frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=3, origin=np.array([0., 0., 0.]))
    entities.append(point_cloud)
    entities.append(frame)
    entities.append(table)
 
    o3d.visualization.draw_geometries(entities,
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

    #o3d.io.write_point_cloud(files_path + '/rgbd-scenes-v2/pc/01_without_ground.ply', plane_points, write_ascii=False, compressed=False, print_progress=False)

   

if __name__ == "__main__":
    main()