#!/usr/bin/env python3

import rospy
from open3d_ros_helper import open3d_ros_helper as orh
import open3d as o3d
from sensor_msgs.point_cloud2 import PointCloud2

def main():
    point_cloud_msg = rospy.Subscriber("/camera/depth_registered/points", PointCloud2)

    o3dpc = orh.rospc_to_o3dpc(point_cloud_msg) 

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
    o3d.visualization.draw_geometries(point_cloud_msg,
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

if __name__ == "__main__":
    main()