#!/usr/bin/env python3

# --------------------------------------------------
# Group 3 - Dora The Mug Finder
# André Cardoso, Fábio Sousa, Tatiana Resende
# SAVI, January 2023.
# --------------------------------------------------

# This code convert a point cloud to open3d format

# Other Packages imports
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from ctypes import * # convert float to uint32
import numpy as np 
import rospy

###########################################
# Initial visualization parameters        #
# (delete in the future)                  #
###########################################
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

###########################################
# Initialization                          #
###########################################
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


class Cloud:
    def __init__(self):
        self.pub = rospy.Publisher('cloud_test', PointCloud2, queue_size=10)

    def callback(self,data):

        #self.pub.publish(data)
        self.convertCloudFromRosToOpen3d(data)

    def convertCloudFromRosToOpen3d(self,ros_cloud):
        
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, field_names = field_names))
        print(len(cloud_data))
        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # Combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
           
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        
        ###########################################
        # Visualization and point cloud writing   #
        # (delete in the future)                  #
        ###########################################
        print(open3d_cloud)
        o3d.visualization.draw_geometries([open3d_cloud],
                                        zoom=view['trajectory'][0]['zoom'],
                                        front=view['trajectory'][0]['front'],
                                        lookat=view['trajectory'][0]['lookat'],
                                        up=view['trajectory'][0]['up'])
        
        o3d.io.write_point_cloud('kinect_complete.ply', open3d_cloud, write_ascii=False, compressed=False, print_progress=False)
   
###########################################
# Execution                               #
###########################################
def main():
    cloud = Cloud()
    rospy.init_node("Cloud_open3d")
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2,cloud.callback)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("ola")
        rate.sleep() 

if __name__ == "__main__":
        main()