#!/usr/bin/env python3
# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# Fábio Sousa.
# SAVI, December 2022.
# --------------------------------------------------

import open3d as o3d
import glob
import random
import numpy as np

def main():
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

    entities=[]
    # filename ='rgbd-scenes-v2/pc/01.ply'
    dataset_path ='rgbd-scenes-v2/pc/03.ply'
    # image_filenames = glob.glob(dataset_path + '/*.ply')
    # image_filenames = random.sample(image_filenames, k=1)
    
    # for image_filename in image_filenames:

    # Sample only a few images to speed up development
    pcd = o3d.io.read_point_cloud(dataset_path)
    # pcd.colors = o3d.utility.Vector3dVector(pcd.color.astype(np.float) / 255.0) #Trying to convert color of point
    entities.append(pcd)
    #Draws entities and show PointCloud in the defined view
    o3d.visualization.draw_geometries(entities,
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])


if __name__ == "__main__":
    main()
