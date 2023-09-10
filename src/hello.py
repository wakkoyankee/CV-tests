import open3d as o3d

# Create a point cloud
points = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)
point_cloud.colors = o3d.utility.Vector3dVector(colors)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])
