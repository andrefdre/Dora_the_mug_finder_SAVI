#!/usr/bin/env python3
# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------
# This code converts a ROS PointCloud2 message to Open3d format and sends a message with the converted point cloud

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from ctypes import * # To convert float to uint32
import numpy as np
from dora_the_mug_finder_msg.msg import Kinect_ply
from geometry_msgs.msg import Point
from colorama import Fore, Style

def main():

    # ---------------------------------
    # Initialization 
    # ---------------------------------
    # Functions to convert RGB values from uint32 or float to tuple
    convert_rgbUint32_to_tuple = lambda rgb_uint32: ((rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))
    convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))

    # Class instantiation
    cloud = Cloud(convert_rgbFloat_to_tuple, convert_rgbUint32_to_tuple)

    # Initialization of a ROS node
    rospy.init_node("ros_to_open3d")

    # Initialization of the subscriber of the PointCloud2 message 
    topic_name =  "/camera/depth_registered/points"
    rospy.Subscriber(topic_name, PointCloud2, cloud.callback) # cloud.callback sends the message to be processed in the class
    
    # Message frequency in Hertz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #print(Style.DIM, Fore.LIGHTBLACK_EX, "Receiving PointCloud from RGBD camera for conversion process", Style.RESET_ALL)
        rate.sleep()

    # ---------------------------------
    # Execution 
    # ---------------------------------

class Cloud:
    
    def __init__(self, convert_rgbFloat_to_tuple, convert_rgbUint32_to_tuple):

        # Create the publisher 
        self.pub = rospy.Publisher("open3d_cloud", Kinect_ply, queue_size=10)
        self.convert_rgbFloat_to_tuple = convert_rgbFloat_to_tuple
        self.convert_rgbUint32_to_tuple = convert_rgbUint32_to_tuple

    def callback(self, data):

        self.convertCloudFromRosToOpen3d(data)

    def convertCloudFromRosToOpen3d(self,ros_cloud):

        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check if it's empty
        open3d_cloud = o3d.geometry.PointCloud()

        if len(cloud_data)==0:
            print(Style.BRIGHT, Fore.YELLOW, "Received Point Cloud message is empty", Style.RESET_ALL)
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb

            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ]

            # Get RGB
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float:
                rgb = [self.convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            else:
                rgb = [self.convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # Combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)

        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        kinect = Kinect_ply()
        # Append point to the correct message field
        for pt in open3d_cloud.points:
            pt = Point(pt[0], pt[1], pt[2]) 
            kinect.point.append(pt)

        # Append color to the correct message field
        for clr in open3d_cloud.colors:
            clr = Point(clr[0], clr[1], clr[2])
            kinect.rgb.append(clr)

    # ---------------------------------
    # Termination
    # ---------------------------------

        self.pub.publish(kinect)

        print(Style.BRIGHT, Fore.GREEN, 'Everything is okay, I am just sending the converted point cloud for processing', Style.RESET_ALL)


if __name__ == "__main__":
    try:
        main()
    except:
        print(Style.BRIGHT, Fore.RED, "Nothing's going on, please correct what's wrong so I can be of use", Style.RESET_ALL)