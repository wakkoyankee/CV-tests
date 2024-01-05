import open3d as o3d
import Dataset as ds
from ReconstructionEngine import ReconstructionEngine
from projection import *
from Detection import detect


def example_screenshot_sequence(obj, im_path):
    #! start visualizer
    scene = o3d.visualization.Visualizer()
    scene.create_window()
    scene.add_geometry(obj)
    scene.update_renderer()
    scene.poll_events()
    view_control = scene.get_view_control()
    view_control.set_zoom(0.2)

    #! movement sequence
    for i in range(40):
        # move camera
        view_control.rotate(40.0, 0.0)
        # get cameraparameters
        cam = view_control.convert_to_pinhole_camera_parameters()
        scene.update_geometry(obj)
        scene.update_renderer()
        scene.poll_events()
        scene.capture_screen_image(im_path + "output_image"+str(i)+".png")

def example_lidar_reconstruction_sequence(pcd, outpath):
    scene = o3d.visualization.Visualizer()
    scene.create_window()
    scene.add_geometry(pcd)
    scene.update_renderer()
    scene.poll_events()
    
    view_control = scene.get_view_control()
    view_control.set_zoom(0.2)

    save_traj = []
    save_rgbds = []

    for i in range(40):
        # move camera
        view_control.rotate(40.0, 0.0)
        
        # get cameraparameters
        cam = view_control.convert_to_pinhole_camera_parameters()
        
        rgbd = project_to_rgbd( pcd,
                                cam.intrinsic.width,
                                cam.intrinsic.height,
                                cam.intrinsic.intrinsic_matrix,
                                cam.extrinsic,
                                1000,
                                10)
        save_rgbds.append(rgbd)
        save_traj.append(cam)
        
        scene.update_geometry(mesh)
        scene.update_renderer()
        scene.poll_events()
        
    reconstruction = ReconstructionEngine()
    pcdlive = reconstruction.reconstruct2pcdlive(save_rgbds, save_traj)
    o3d.io.write_point_cloud(outpath, pcdlive)


if __name__ == "__main__":
    #! Paths
    scene_mesh_path = "Data/scene1.ply"
    scene_pcd_path = "Data/scene1.ply"
    
    screenshot_path = "image/"
    lidar_path = "data/output.ply"
    
    model_path = "/Users/hadrienrothea/Desktop/codes/CV-tests/efficientdet.tflite"
    image_path = "/Users/hadrienrothea/Desktop/codes/CV-tests/image/output_image35.png"
    
    #! To get a preset 3D scene from open3D
    #ds.create_orignal_scene(scene_path, False)
    
    #! If already created or you have object use either
    mesh = o3d.io.read_triangle_mesh(scene_mesh_path)
    pcd = o3d.io.read_point_cloud(scene_pcd_path)
    
    #! Screenshot example
    #example_screenshot_sequence(mesh, screenshot_path)
    
    #! Lidar reconstruction example
    # for speed can use a downsampled pointcloud
    #tmppcd = mesh.sample_points_poisson_disk(20000)
    #example_lidar_reconstruction_sequence(pcd, lidar_path)
    
    #! Detection example
    detect(model_path, image_path)
