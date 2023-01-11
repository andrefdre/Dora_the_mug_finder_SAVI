import numpy as np
import open3d as o3d

class Object_detection():    
    def __init__(self):
        super().__init__()
        pass

    def bbox(self,table):
    
        obb = table.get_oriented_bounding_box().get_box_points()

        #First create a point cloud with the vertices of the desired bounding box (it has 8 vertices in 3 dimensions)
        np_points = np.asarray(obb)
        #print(np_points)
        #The bbox has it's center on the origin, therefore each vertice will be a combination of x, y and z max and min values
        idx = 0
        while True:

            if np_points[idx,2] > 0:
                np_points[idx,2] = np_points[idx,2]+0.4
            # else:
            #     np_points[idx,2] = -0.02
    
            idx += 1

            if idx == 8:
                break
            
        #Convert from numpy to Open3D
        bbox_points = o3d.utility.Vector3dVector(np_points) 

        #Create Axis Aligned Bounding Box from points
        bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(bbox_points)
        bbox.color = (1, 0, 0) #Colors bbox lines in red

        return bbox