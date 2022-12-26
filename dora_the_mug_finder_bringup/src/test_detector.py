#!/usr/bin/env python3

import open3d as o3d
import rospkg

#! Scene is opening with rainbow color instead of normal color

# find dora_the_mug_finder_bringup folder directory
rospack = rospkg.RosPack()
rospack.list()
path = rospack.get_path('dora_the_mug_finder_bringup')


view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 2.611335277557373, 1.2635015249252319, 3.83980393409729 ],
			"boundingbox_min" : [ -2.5246021747589111, -1.5300980806350708, -1.4928504228591919 ],
			"field_of_view" : 60.0,
			"front" : [ 0.24529435406824474, -0.14922644993126083, -0.95789464269467317 ],
			"lookat" : [ -0.81991542304731513, 0.74917048653234597, 3.9091168099353029 ],
			"up" : [ -0.20684927135282513, -0.97338458361137858, 0.098670316349491194 ],
			"zoom" : 0.73999999999999977
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}


def main():

    # ------------------------------------------
    # Initialization
    # ------------------------------------------
    print("Load a ply point cloud, print it, and render it")

    point_cloud = o3d.io.read_point_cloud(path + '/src/abc.ply')

    o3d.visualization.draw_geometries([point_cloud],
                                    zoom=view['trajectory'][0]['zoom'],
                                    front=view['trajectory'][0]['front'],
                                    lookat=view['trajectory'][0]['lookat'],
                                    up=view['trajectory'][0]['up'])

    # ------------------------------------------
    # Execution
    # ------------------------------------------

    # ------------------------------------------
    # Termination
    # ------------------------------------------
  
    
if __name__ == "__main__":
    main()