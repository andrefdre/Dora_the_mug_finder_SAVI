#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------

# General Imports 
import threading
import time
from more_itertools import locate
from colorama import Fore, Style
from copy import deepcopy
from math import sqrt
import open3d as o3d
import open3d.visualization.gui as gui
import numpy as np
import argparse
import sys
import os
import rospy
import glob
from std_msgs.msg import String

# Oun Package imports
from dora_the_mug_finder_msg.msg import Object , Point , Classes
from dora_the_mug_finder_bringup.src.table_detection import PlaneDetection, PlaneTable, Table, Transform
from dora_the_mug_finder_bringup.src.utils import text_3d

# Stores the view 
view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 1.2571436490455929, 0.5, 2.8718910217285156 ],
			"boundingbox_min" : [ -1.5309394598007202, -2.0348093509674072, -0.029999999999999999 ],
			"field_of_view" : 60.0,
			"front" : [ 0.11555556622862719, 0.11345331420937423, -0.98680051510347855 ],
			"lookat" : [ -0.23064077816298553, -0.2045093977126011, 0.17408966530741635 ],
			"up" : [ 0.013560659994457255, -0.99354326124308379, -0.11264056347059027 ],
			"zoom" : 0.1
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}



# Class that will deal with ROS messages
class ROSHandler:

    def __init__(self,files_path):
        self.update_names = False
        self.object_names = []
        self.files_path = files_path
        self.scene_name = 'scene_01'
        self.filename = self.files_path + '/rgbd-scenes-v2/pc/01.ply'
        os.system('pcl_ply2pcd ' + self.filename + ' pcd_point_cloud.pcd')
        self.point_cloud_original = o3d.io.read_point_cloud('pcd_point_cloud.pcd')
                            

    def callback_class(self,data):
        self.object_names = [name_string.data for name_string in data.classes]

    def callback_scene(self,data):
        if data.data=='kinect':
            self.scene_name = data.data
        else:
            self.scene_name = data.data
            scene_number = self.scene_name.split('_')
            self.filename = self.files_path + f'/rgbd-scenes-v2/pc/{scene_number[-1]}.ply'
            os.system('pcl_ply2pcd ' + self.filename + ' pcd_point_cloud.pcd')
            self.point_cloud_original = o3d.io.read_point_cloud('pcd_point_cloud.pcd')



# Class that will deal with visualization
class Visualize:
    def __init__(self,vis):
        self.vis = vis
        self.flag_play = True

    def visualize(self):
        self.vis.poll_events()
        self.vis.update_renderer()
        self.vis.run()


    def space_callback(self, vis):
        if self.flag_play:
            print('Playback paused, press [Space] to continue.')
        else:
            print('Playback resumed, press [Space] to pause.')
        self.flag_play = not self.flag_play
        return False


def main():
    ###########################################
    # Initialization                          #
    ###########################################
    parser = argparse.ArgumentParser(description='Data Collector')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Visualize the point cloud')
    
    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(parser.parse_args(args=arglist))

    ###########################################
    # Ros Initialization                      #
    ###########################################
    files_path=f'{os.environ["DORA"]}'

    ros_handler = ROSHandler(files_path)

    pub = rospy.Publisher('objects_publisher', Object, queue_size=10)
    rospy.Subscriber("class_publisher", Classes, ros_handler.callback_class)
    rospy.Subscriber("scene_publisher", String, ros_handler.callback_scene)
    rospy.init_node('objects', anonymous=False)
    rate = rospy.Rate(10) # 10hz

    ###########################################
    # Object detection Initialization         #
    ###########################################    
    scenes_number_objects = {
        '01': 5,
        '02': 5,
        '03': 5,
        '04': 5,
        '05': 4,
        '06': 5,
        '07': 5,
        '08': 4,
        '09': 3,
        '10': 3,
        '11': 3,
        '12': 3,
        '13': 4,
        '14': 4,
        'kinect' : 0
    }

    vis = o3d.visualization.VisualizerWithKeyCallback()
    visualizer = Visualize(vis)
    vis.register_key_callback(32, visualizer.space_callback)
    
    vis.create_window("Scene")
    ctr = vis.get_view_control()
    vis.add_geometry(ros_handler.point_cloud_original)
    ctr.set_front(view['trajectory'][0]['front'])
    ctr.set_up(view['trajectory'][0]['up'])
    ctr.set_lookat(view['trajectory'][0]['lookat'])
    ctr.set_zoom(view['trajectory'][0]['zoom'])


    while not rospy.is_shutdown():
        ########################################
        # Gets the scene number and nº objects #
        ########################################
        if visualizer.flag_play:
            ####################################
            # Initialization of loop           #
            ####################################
            parts = ros_handler.filename.split('/')
            part = parts[-1]
            parts = part.split('.')
            scene_number = parts[0]
            scene_number_objects = scenes_number_objects[scene_number]

            # Creates copy of the information coming from ros to prevent changing during middle of the code
            point_cloud_original = deepcopy(ros_handler.point_cloud_original)  
            scene_name = ros_handler.scene_name
            ########################################
            # Find two planes                      #
            ########################################
            # find two planes: table and another     
            point_cloud_twoplanes = deepcopy(point_cloud_original) 
            number_of_planes = 2
            planes = []
            while True: # run consecutive plane detections

                plane = PlaneDetection(point_cloud_twoplanes) # create a new plane instance
                point_cloud_twoplanes = plane.segment() # new point cloud are the outliers of this plane detection
        
                planes.append(plane)

                if len(planes) >= number_of_planes: # stop detection planes
                    break

            ########################################
            # Find table plane                     #
            ########################################
            plane_table = PlaneTable(planes) # create a new plane_table instance
            plane_table = plane_table.planetable()  

            # definition point cloud without table and point cloud only table
            plane_table.outlier_cloud = plane_table.outlier_cloud.voxel_down_sample(voxel_size=0.005)
            
            ########################################
            # Find plane                           #
            ########################################
            t = Table()     # object initialization 
            t.voxel_down_sample(plane_table.inlier_cloud,voxel_size=0.005)
            t.cluster(t.down_sampled_plane)
            t.table(t.cluster_idxs, t.object_idxs, t.down_sampled_plane)

            ########################################
            # Frame alignment                      #
            ########################################
            frame = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
            tx, ty, tz = t.table.get_center()
            x, y, z = plane_table.a, plane_table.b, plane_table.c
            
            plane_table.outlier_cloud = Transform(0,0,0,-tx,-ty,-tz).translate(plane_table.outlier_cloud)
            t.table = Transform(0,0,0,-tx,-ty,-tz).translate(t.table)
            
            plane_table.outlier_cloud = Transform(-x,-y,-z,0,0,0).rotate(plane_table.outlier_cloud)
            t.table = Transform(-x,-y,-z,0,0,0).rotate(t.table)

            ########################################
            # Objects Detection                    #
            ########################################
            # bbox: table + objects
            t.bbox_table(t.table)

            # objects + noise
            point_cloud_objects_noise = plane_table.outlier_cloud.crop(t.bbox)
    
            # objects
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

                d['bbox_obj'] = d['points'].get_axis_aligned_bounding_box()
                d['bbox_to_draw'] = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(d['bbox_obj'])

                if d['z'] > threshold_z and dist < threshold_dist and d['width'] < threshold_width and d['length'] < threshold_length:       
                    # condition of being object: Z center > 0, be close to the reference, not be too big
                    objects.append(d) #Add the dict of this object to the list


            #####################################
            # BBox extraction                   #
            #####################################
            entities = []
            objects_3d = Object()
            for object_idx, object in enumerate(objects):
                object['points'] = Transform(-x,y,z,0,0,0).rotate(object['points'],inverse=True)
                object['points'] = Transform(0,0,0,tx,ty,tz).translate(object['points'])

                object['bbox_to_draw'] = Transform(-x,y,z,0,0,0).rotate(object['bbox_to_draw'],inverse=True)
                bbox_to_draw = Transform(0,0,0,tx,ty,tz).translate(object['bbox_to_draw'])
                
                # Create ROS Message #
                # Gets the min and max of the bounding box
                min = bbox_to_draw.get_min_bound() 
                objects_3d.corners.append(Point(min[0],min[1],min[2]))
                max = bbox_to_draw.get_max_bound()
                objects_3d.corners.append(Point(max[0],max[1],max[2]))
                # Gets the center points
                center = object['points'].get_center()
                objects_3d.center.append(Point(center[0],center[1],center[2]))
                objects_3d.scene = String(scene_name)

                # Creates the entities to be drawn
                sphere =o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
                sphere.paint_uniform_color([1.0, 0.75, 0.0])
                sphere.translate(center)
                if args['visualize']: # Checks if the user wants to visualize the point cloud
                    entities.append(sphere)
                    entities.append(bbox_to_draw)
                    if len(ros_handler.object_names) == len(objects):
                        text = f'{ros_handler.object_names[object_idx]}'
                        object_name = text_3d(text , font_size=20)
                        object_name = Transform(-x,y,z,0,0,0).rotate(object_name,letter=True)
                        object_name = Transform(0,0,0,center[0]-0.1,center[1]-0.2,center[2]).translate(object_name)
                        object_name.paint_uniform_color([1.0, 0.75, 0.0])
                        entities.append(object_name)


            ######################################
            # Visualization                      #
            ######################################
            if scene_number_objects != len(objects):
                print(Fore.RED + 'number of objects is wrong' + Style.RESET_ALL)

        if args['visualize'] and visualizer.flag_play: # Checks if the user wants to visualize the point cloud
            text = f'number of objects: {scene_number_objects}'
            text_number_objects = text_3d(text , font_size=20)
            text_number_objects = Transform(-x,y,z,0,0,0).rotate(text_number_objects,letter=True)
            text_number_objects = Transform(0,0,0,tx-1,ty-1.3,tz).translate(text_number_objects)
            #entities.append(t.bbox)
            entities.append(text_number_objects)
            entities.append(frame)
            # Displays the entities
            vis.clear_geometries()
            vis.add_geometry(point_cloud_original,reset_bounding_box=False)
            for entity in entities:
                vis.add_geometry(entity,reset_bounding_box=False)

        visualizer.visualize()


        #################################
        # ROS Publish                   #
        #################################
        pub.publish(objects_3d) # Publishes the detected objects
        rate.sleep() # Sleeps to if time < rate


    

if __name__ == "__main__":
    main()