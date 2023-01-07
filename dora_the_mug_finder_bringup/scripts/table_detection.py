from more_itertools import locate

class PlaneDetection():
    def __init__(self, point_cloud):

        self.point_cloud = point_cloud

    def colorizeInliers(self, r,g,b):
        self.inlier_cloud.paint_uniform_color([r,g,b]) # paints the plane in red

    def segment(self, distance_threshold=0.03, ransac_n=5, num_iterations=50):

        print('Starting plane detection')
        self.plane_model, inlier_idxs = self.point_cloud.segment_plane(distance_threshold=distance_threshold, 
                                                    ransac_n=ransac_n,
                                                    num_iterations=num_iterations)
        [self.a, self.b, self.c, self.d] = self.plane_model

        self.inlier_cloud = self.point_cloud.select_by_index(inlier_idxs)
        outlier_cloud = self.point_cloud.select_by_index(inlier_idxs, invert=True)
        return outlier_cloud


    def __str__(self):
        text = 'Segmented plane from pc with ' + str(len(self.point_cloud.points)) + ' with ' + str(len(self.inlier_cloud.points)) + ' inliers. '
        text += '\nPlane: ' + str(self.a) +  ' x + ' + str(self.b) + ' y + ' + str(self.c) + ' z + ' + str(self.d) + ' = 0' 
        return text

class PlaneTable():
    def __init__(self, planes):
        self.planes = planes

    def planetable(self):
        distd = abs(self.planes[0].d) - abs(self.planes[1].d)
        print('dist_d=' + str(abs(distd)))
        
        if abs(distd) > 0.3:
            print('plano da mesa e plano chao')
            if distd > 0:
                plane = self.planes[1]
            else:
                plane = self.planes[0]     
        else:
            plane = self.planes[0]
            print('plano mesa e outro')
        
        return plane

    
class Table():
    def __init__(self):
        super().__init__()
        pass

    def voxel_dow_sample(self,plane,voxel_size=0.005):
        plane = plane.voxel_down_sample(voxel_size=voxel_size)
        
        return plane

    def cluster(self,plane,eps=0.035, min_points=60, print_progress=True):
        cluster_idxs = list(plane.cluster_dbscan(eps=eps, min_points=min_points, print_progress=print_progress))
        object_idxs = list(set(cluster_idxs))
        object_idxs.remove(-1) #Removes -1 cluster ID (-1 are the points not clustered)

        return cluster_idxs, object_idxs

    def table(self,cluster_idxs, object_idxs,plane):
        table = []
        num_points = 0
        num_points_list = []

        for object_idx in object_idxs:
            object_point_idxs = list(locate(cluster_idxs, lambda x: x == object_idx))
            object_points = plane.select_by_index(object_point_idxs)
            object_points.paint_uniform_color([0, 1, 0])
        
            num_points_new = len(object_points.points)
            if num_points_new > num_points:
                table = object_points
                print('nº point obj 0: ' + str(len(object_points.points)))

            num_points_list.append(num_points_new)
            num_points = max(num_points_list)
        
        return table
    





        
    
