import open3d as o3d
import numpy as np

def conver_points_to_pcd (pts) -> o3d.geometry.PointCloud :
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd