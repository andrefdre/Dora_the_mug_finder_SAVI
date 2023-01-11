from more_itertools import locate
import open3d as o3d
import numpy as np
from math import sqrt, pi

class PlaneDetection():
    def __init__(self, point_cloud):

        self.point_cloud = point_cloud

    def colorizeInliers(self, r,g,b):
        self.inlier_cloud.paint_uniform_color([r,g,b]) # paints the plane in red

    def segment(self, distance_threshold=0.035, ransac_n=5, num_iterations=50):

        #print('Starting plane detection')
        self.plane_model, self.inlier_idxs = self.point_cloud.segment_plane(distance_threshold=distance_threshold, 
                                                    ransac_n=ransac_n,
                                                    num_iterations=num_iterations)
        # distance_threshold (float) – Max distance a point can be from the plane model, and still be considered an inlier.
        # ransac_n (int) – Number of initial points to be considered inliers in each iteration.
        # num_iterations (int) – Number of iterations.


        [self.a, self.b, self.c, self.d] = self.plane_model

        self.inlier_cloud = self.point_cloud.select_by_index(self.inlier_idxs)
        self.outlier_cloud = self.point_cloud.select_by_index(self.inlier_idxs, invert=True)
        return self.outlier_cloud


    def __str__(self):
        text = 'Segmented plane from pc with ' + str(len(self.point_cloud.points)) + ' with ' + str(len(self.inlier_cloud.points)) + ' inliers. '
        text += '\nPlane: ' + str(self.a) +  ' x + ' + str(self.b) + ' y + ' + str(self.c) + ' z + ' + str(self.d) + ' = 0' 
        return text

class PlaneTable():
    def __init__(self, planes):
        self.planes = planes
        #self.inlieres = inlieres

    def planetable(self):
        distd = abs(self.planes[0].d) - abs(self.planes[1].d)
        # print('dist_d=' + str(abs(distd)))
        
        if abs(distd) > 0.3:
            # print('plano da mesa e plano chao')
            if distd > 0:
                self.plane = self.planes[1]
                #inlier = self.inlieres[1]

            else:
                self.plane = self.planes[0]
                #inlier = self.inlieres[0]     
        else:
            self.plane = self.planes[0]
            #inlier = self.inlieres[0]
           # print('plano mesa e outro')
        
        return self.plane#, inlier

    
class Table():
    def __init__(self):
        super().__init__()
        pass

    def voxel_dow_sample(self,plane,voxel_size=0.005):
        self.down_sampled_plane = plane.voxel_down_sample(voxel_size=voxel_size)

    def cluster(self,plane,eps=0.035, min_points=60, print_progress=True):
        self.cluster_idxs = list(plane.cluster_dbscan(eps=eps, min_points=min_points, print_progress=print_progress))
        self.object_idxs = list(set(self.cluster_idxs))
        self.object_idxs.remove(-1) #Removes -1 cluster ID (-1 are the points not clustered)

        #return cluster_idxs, object_idxs

    def table(self,cluster_idxs, object_idxs,plane):
        self.table = []
        inlier_idxs = []
        num_points = 0
        num_points_list = []

        for object_idx in object_idxs:
            object_point_idxs = list(locate(cluster_idxs, lambda x: x == object_idx))
            object_points = plane.select_by_index(object_point_idxs)
            object_points.paint_uniform_color([0, 1, 0])
        
            num_points_new = len(object_points.points)
            if num_points_new > num_points:
                self.table = object_points
                # print('nº point obj 0: ' + str(len(object_points.points)))

            num_points_list.append(num_points_new)
            num_points = max(num_points_list)
        

    def bbox_table(self,table_algn):
        self.table_algn = table_algn
        obb = self.table_algn.get_oriented_bounding_box().get_box_points()

        #First create a point cloud with the vertices of the desired bounding box (it has 8 vertices in 3 dimensions)
        np_points = np.asarray(obb)
        
        #The bbox has it's center on the origin, therefore each vertice will be a combination of x, y and z max and min values
        idx = 0
        while True:

            if np_points[idx,2] > 0:
                np_points[idx,2] = np_points[idx,2]+0.4
    
            idx += 1

            if idx == 8:
                break
            
        #Convert from numpy to Open3D
        bbox_points = o3d.utility.Vector3dVector(np_points) 

        #Create Axis Aligned Bounding Box from points
        self.bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(bbox_points)
        self.bbox.color = (1, 0, 0) #Colors bbox lines in red


class Transform():    
    def __init__(self,x,y,z,tx,ty,tz):
        self.x = x
        self.y = y
        self.z = z
        self.tx = tx
        self.ty = ty
        self.tz = tz

    def rotate(self,cloud_points):        
        num = np.dot([self.x,self.y,self.z],[0,0,1])
        dem = abs(sqrt(self.x**2+self.y**2+self.z**2))
        xAngle = num/dem
        yAngle = 0 
        zAngle = 0
        r = xAngle + pi/2 + pi
        p = yAngle 
        y = zAngle   
    
        rotation = cloud_points.get_rotation_matrix_from_xyz((r,p,y))
        cloud_points.rotate(rotation, center=( self.tx, self.ty, self.tz))
        return cloud_points

    def translate(self,cloud_points):
        cloud_points.translate((self.tx,self.ty,self.tz))
        return cloud_points