import open3d as o3d
import Trajectory as tj
import numpy as np

VOXEL_LENGTH = 4.0 / 512.0
SDF_TRUNC = 0.04
COLOR_TYPE = o3d.pipelines.integration.TSDFVolumeColorType.RGB8

class ReconstructionEngine:
    
    def __init__(self, voxel_length=VOXEL_LENGTH, sdf_trunc=SDF_TRUNC, color_type=COLOR_TYPE) -> None:
        self.voxel_length = voxel_length
        self.sdf_trunc = sdf_trunc
        self.color_type = color_type

    def reconstruct(self,rgbds, camera_poses):
        
        # Prepare TSDF volumne function
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=self.color_type)

        # Read RGBDs and camera poses
        for i in range(len(camera_poses)):
            print("Integrate {:d}-th image into the volume.".format(i))
            volume.integrate(
                rgbds[i],
                o3d.camera.PinholeCameraIntrinsic(
                    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
                np.linalg.inv(camera_poses[i].pose))
            
        return volume
    
    def reconstructlive(self, rgbds, cameras):
        # Prepare TSDF volumne function
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=self.color_type)

        # Read RGBDs and camera poses
        for i in range(len(rgbds)):
            print("Integrate {:d}-th image into the volume.".format(i))      
            volume.integrate(
                rgbds[i],
                cameras[i].intrinsic,
                cameras[i].extrinsic)
            
        return volume
    

    def reconstruct2mesh(self,rgbds, camera_poses):
        #Get volume
        volume = self.reconstruct(rgbds, camera_poses)
        mesh = volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        return mesh
    
    def reconstruct2pcd(self,rgbds, camera_poses):
        #Get volume
        volume = self.reconstruct(rgbds, camera_poses)
        pcd = volume.extract_point_cloud()
        return pcd
    
    def reconstruct2pcdlive(self,rgbds, cameras):
        #Get volume
        volume = self.reconstructlive(rgbds, cameras)
        pcd = volume.extract_point_cloud()
        return pcd