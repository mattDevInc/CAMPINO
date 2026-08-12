import open3d as o3d
import numpy as np
import networkx as nx

def convert_points_to_pcd (pts) -> o3d.geometry.PointCloud :
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd

def line_set_from_graph (graph_) -> o3d.geometry.LineSet :
    pts = []
    for i in range(len(graph_.nodes)) :
        pts.append(graph_.nodes[i]["center"])

    lines = [l for l in graph_.edges]
    l_set = o3d.geometry.LineSet(points = o3d.utility.Vector3dVector(pts), lines = o3d.utility.Vector2iVector(lines))
    return l_set