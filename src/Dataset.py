import open3d as o3d
from ReconstructionEngine import ReconstructionEngine
import Trajectory as tj
import os

def create_orignal_scene(path, ismesh):
    #Get dataset
    dataset = o3d.data.RedwoodIndoorLivingRoom1()


    #Read RGBDs
    im_rgbds = []
    for color_path, depth_path in zip(dataset.color_paths, dataset.depth_paths):
        im_color = o3d.io.read_image(color_path)
        im_depth = o3d.io.read_image(depth_path)
        im_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            im_color, im_depth,depth_trunc=4.0, convert_rgb_to_intensity=False)
        im_rgbds.append(im_rgbd)
        

    #Read trajectory
    trajectory_path = dataset.trajectory_path
    camera_poses = tj.read_trajectory(trajectory_path)
    
    reconstruction = ReconstructionEngine()
    if ismesh:
        mesh = reconstruction.reconstruct2mesh(im_rgbds, camera_poses)
        o3d.io.write_triangle_mesh(path, mesh)
    else:
        pcd = reconstruction.reconstruct2pcd(im_rgbds, camera_poses)
        o3d.io.write_triangle_mesh(path, mesh)


    
    