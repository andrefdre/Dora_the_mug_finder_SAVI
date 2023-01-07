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

    
    
   





        
    
