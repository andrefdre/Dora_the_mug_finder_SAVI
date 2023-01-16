#!/usr/bin/env python3

import rospy
import cv2
# from functools import partial
# from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.point_cloud2 import PointCloud2
import open3d as o3d
import pcl

class image:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.subscriberCallback)
        self.image_args = {} # Contains cv_image
        # self.point_cloud_msg = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.ros_to_pcl)
        # self.point_cloud_msg = rospy.Subscriber("/camera/depth_registered/points", PointCloud2)
        self.begin_image = False

    def subscriberCallback(self,image_msg):

        self.begin_image = True # After receiving first image will be forever true

        #! If I assign directly to cv_image key, it will show flickers of the non flipped image 
        self.image_args["cv_image"] = self.bridge.imgmsg_to_cv2(image_msg,"bgr8") # Passthrough means wtv the encoding was before, it will be retained
        # rospy.loginfo("Received image message, image shape is " + str(self.image_args["cv_image"].shape))

    def showImage(self):
        # view = {
        #     "class_name" : "ViewTrajectory",
        #     "interval" : 29,
        #     "is_loop" : False,
        #     "trajectory" :
        #     [
        #         {
        #             "boundingbox_max" : [ 0.90000000000000002, 0.90000000000000002, 0.5 ],
        #             "boundingbox_min" : [ -0.90000000000000002, -0.90000000000000002, -0.10000000000000003 ],
        #             "field_of_view" : 60.0,
        #             "front" : [ 0.57428340315845261, 0.72415504342386261, 0.38183510307530683 ],
        #             "lookat" : [ -0.23064077816298553, -0.2045093977126011, 0.17408966530741635 ],
        #             "up" : [ -0.36479928025136604, -0.19118507801017759, 0.91124626258455921 ],
        #             "zoom" : 0.79999999999999893
        #         }
        #     ],
        #     "version_major" : 1,
        #     "version_minor" : 0
        #     }

        if not self.begin_image: # Only shows after first image is processed
            return
        # print(type(self.point_cloud_msg))
        cv2.namedWindow("cv_image")
        cv2.imshow("cv_image", self.image_args["cv_image"])
        pressed_key = cv2.waitKey(1) & 0xFF # To prevent NumLock issue

        if pressed_key == ord('q'):
            print("Quitting program")
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Order to quit") # Stops ros
            exit()    
        
        # o3d.visualization.draw_geometries(self.point_cloud_msg,
        #                                 zoom=view['trajectory'][0]['zoom'],
        #                                 front=view['trajectory'][0]['front'],
        #                                 lookat=view['trajectory'][0]['lookat'],
        #                                 up=view['trajectory'][0]['up'])
            
    # def ros_to_pcl(self, point_cloud_msg):
    #     """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

    #         Args:
    #             point_cloud_msg (PointCloud2): ROS PointCloud2 message

    #         Returns:
    #             pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    #     """
    #     points_list = []

    #     for data in PointCloud2.read_points(point_cloud_msg, skip_nans=True):
    #         points_list.append([data[0], data[1], data[2], data[3]])

    #     pcl_data = pcl.PointCloud_PointXYZRGB()
    #     pcl_data.from_list(points_list)
    #     print(points_list)


def main():
    rospy.init_node("camera_footage", anonymous=False)
    bottom_front_camera = image()

    while True:
        bottom_front_camera.showImage()


if __name__ == "__main__":
    main()