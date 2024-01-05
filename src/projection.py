import open3d as o3d
import numpy as np

# intri 3x3, extr 4x4
def project_to_rgbd(cloud,
                    width,
                    height,
                    intrinsic,
                    extrinsic,
                    depth_scale,
                    depth_max
                    ):
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
    n = len(points)
    
    depth = np.zeros((height, width, 1), dtype=np.uint16)
    color = np.zeros((height, width, 3), dtype=np.uint8)

    # The commented code here is vectorized but is missing the filtering at the end where projected points are
    # outside image bounds and depth bounds.
    # world_points = np.asarray(self.points).transpose()
    # world_points = np.append(world_points, np.ones((1, world_points.shape[1])), axis=0)
    # points_in_ccs = np.matmul(extrinsic, world_points)[:3]
    # points_in_ccs = points_in_ccs / points_in_ccs[2, :]
    # projected_points = np.matmul(intrinsic, points_in_ccs)
    # projected_points = projected_points.astype(np.int16)

    for i in range(0, n):
        point4d = np.append(points[i], 1)
        new_point4d = np.matmul(extrinsic, point4d)
        point3d = new_point4d[:-1]
        zc = point3d[2]
        new_point3d = np.matmul(intrinsic, point3d)
        new_point3d = new_point3d/new_point3d[2]
        u = int(round(new_point3d[0]))
        v = int(round(new_point3d[1]))

        # Fixed u, v checks. u should be checked for width
        if (u < 0 or u > width - 1 or v < 0 or v > height - 1 or zc <= 0 or zc > depth_max):
            continue

        d = zc * depth_scale
        depth[v, u ] = d
        color[v, u, :] = colors[i] * 255

    im_color = o3d.geometry.Image(color)
    im_depth = o3d.geometry.Image(depth)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        im_color, im_depth, depth_scale=1000.0, depth_trunc=depth_max, convert_rgb_to_intensity=False)
    return rgbd


# Projection test code
# Use unequal length, width and height. 
""" points = np.random.rand(30000, 3) * [1,2,3]
colors = np.random.rand(30000, 3)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)

scene = o3d.visualization.Visualizer()
scene.create_window()
scene.add_geometry(pcd)
scene.update_renderer()
scene.poll_events()
scene.run()
view_control = scene.get_view_control()
cam = view_control.convert_to_pinhole_camera_parameters()
scene.destroy_window()

rgbd = project_to_rgbd(pcd, cam.intrinsic.width,
                  cam.intrinsic.height,
                  cam.intrinsic.intrinsic_matrix,
                  cam.extrinsic,
                  1000,
                  10)
pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic=cam.intrinsic, extrinsic=cam.extrinsic, project_valid_depth_only=True)
scene = o3d.visualization.Visualizer()
scene.create_window()
scene.add_geometry(pcd1)
scene.run() """