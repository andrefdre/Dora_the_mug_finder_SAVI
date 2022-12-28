#!/usr/bin/env python3

import copy
import csv
import math
import pickle
from copy import deepcopy
from random import randint
from turtle import color

import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from point_cloud_processing import PointCloudProcessing
from matplotlib import cm
from more_itertools import locate

#! --------------------------------------------------------------------
import rospkg
rospack = rospkg.RosPack()
rospack.list()
path = rospack.get_path('dora_the_mug_finder_bringup')
print (path)
#! --------------------------------------------------------------------

view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 3.0000000000000004, 3.0000000000000004, 3.83980393409729 ],
			"boundingbox_min" : [ -2.5246021747589111, -1.5300980806350708, -1.4928504228591919 ],
			"field_of_view" : 60.0,
			"front" : [ 0.67118682276199615, -0.70042708582828483, -0.24271412482332699 ],
			"lookat" : [ 0.11570111840900868, -0.033346828483835626, 1.7983664180669476 ],
			"up" : [ -0.73923053281361617, -0.60805019100000812, -0.28950506831651673 ],
			"zoom" : 0.39999999999999969
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

def main():

    # ------------------------------------------
    # Initialization
    # ------------------------------------------
    p = PointCloudProcessing()
    p.loadPointCloud(path + '/src/02.ply')

    # ------------------------------------------
    # Execution
    # ------------------------------------------

    # Ex1
    p.preProcess(voxel_size=0.01)

    # Ex2
    p.transform(-108,0,0,0,0,0)
    p.transform(0,0,-37,0,0,0)
    p.transform(0,0,0,-0.85,-1.10,0.35)

    #Ex3 
    p.crop(-0.9, -0.9, -0.3, 0.9, 0.9, 0.4)

    #Ex4 
    outliers = p.findPlane()
    
    # Ex5 - Clustering
    cluster_idxs = list(outliers.cluster_dbscan(eps=0.03, min_points=60, print_progress=True))
    object_idxs = list(set(cluster_idxs))
    object_idxs.remove(-1)

    number_of_objects = len(object_idxs)
    colormap = cm.Pastel1(list(range(0,number_of_objects)))

    objects = []
    for object_idx in object_idxs:

        object_point_idxs = list(locate(cluster_idxs, lambda x: x == object_idx))
        object_points = outliers.select_by_index(object_point_idxs)
        # Create a dictionary to represent the objects
        d = {}
        d['idx'] = str(object_idx)
        d['points'] = object_points
        d['color'] = colormap[object_idx, 0:3]
        d['points'].paint_uniform_color(d['color']) # paints the plane in red
        d['center'] = d['points'].get_center()
        objects.append(d) # add the dict of this object to the list
  
    # ------------------------------------------
    # Visualization
    # ------------------------------------------
    # Create a list of entities to draw
    p.inliers.paint_uniform_color([0,1,1]) # paints the plane in red
    entities = []

    frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
    entities.append(frame)

    # Draw bbox
    bbox_to_draw = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(p.bbox)
    entities.append(bbox_to_draw)

    # Draw objects
    for object_idx, object in enumerate(objects):
        if object_idx == 3: #  show only object idx = 2
            entities.append(object['points'])

    # o3d.visualization.draw_geometries(entities,
    #                                 zoom=view['trajectory'][0]['zoom'],
    #                                 front=view['trajectory'][0]['front'],
    #                                 lookat=view['trajectory'][0]['lookat'],
    #                                 up=view['trajectory'][0]['up'],
    #                                 point_show_normal=False)

    #! AULA -------------------------------------------------------------------------------


    o3d.geometry.PointCloud.dimension(entities)


if __name__ == "__main__":
    main()
