import cv2
import numpy as np
import open3d as o3d

class Explorer:
    
    def __init__(self, cloud) -> None:
        self.set_cloud(cloud)
        self.set_intrinsic()
        self.set_extrinsic()
        self.set_depthscale()
        self.set_depthmax()
    
    def set_cloud(self, cloud):
        self.cloud = o3d.t.geometry.PointCloud.from_legacy(cloud)
    
    def set_intrinsic(self, width=640, height=480, focal_dist=6064):
        self.width = width
        self.height = height
        self.focal_dist  = focal_dist
        
        self.intrinsic = o3d.core.Tensor([[focal_dist, 0     , width * 0.5], 
                                          [0     , focal_dist, height * 0.5],
                                          [0     , 0     , 1]])
    
    def set_extrinsic(self, x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0):
        self.extrinsic = np.eye(4)
        self.extrinsic[:3,  3] = (x, y, z)
        self.extrinsic[:3, :3] = o3d.geometry.get_rotation_matrix_from_axis_angle(np.radians(np.asarray((rx, ry, rz))))
        
    def set_depthscale(self, depth_scale=15.0):
        self.depth_scale = depth_scale
    
    def set_depthmax(self, depth_max=10.0):
        self.depth_max = depth_max
    
    def get_rgbd(self):
        rgbd = self.cloud.project_to_rgbd_image(self.width,
                                        self.height,
                                        self.intrinsic,
                                        self.extrinsic,
                                        self.depth_scale,
                                        self.depth_max)
        return rgbd
    
    def get_rgb(self):
        return np.asarray(self.get_rgbd().color)
    
    def move(self, order, value):
        if order == 'x':
            pass
        elif order == 'y':
            pass
        elif order == 'z':
            pass
        elif order == 'rx':
            pass
        elif order == 'ry':
            pass
        elif order == 'rz':
            pass
        
        #def turn_around(self, point):

if __name__ == "__main__":
    #Testing
    points = np.random.rand(100000, 3)
    colors = np.random.rand(100000, 3)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    explorer = Explorer(point_cloud)
    x, y, z, = -0.44999999999999996, -0.5, 1260.0
    explorer.set_intrinsic(640, 380, 120000)
    explorer.set_extrinsic(x,y,z)
    explorer.set_depthmax(10000.0)
    explorer.set_depthscale(15.0)
    img = explorer.get_rgb()
    cv2.imshow("depth", img)
    cv2.waitKey(1000000)
