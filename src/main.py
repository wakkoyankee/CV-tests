import open3d as o3d
import Dataset as ds


mesh  = ds.load_original_scene()
o3d.visualization.draw_geometries([mesh])