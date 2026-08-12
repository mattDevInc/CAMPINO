import open3d as o3d

skelelines = o3d.io.read_line_set("./CAMPINO_skel.ply")
tube = o3d.io.read_point_cloud("./cropped_downsampled.pcd")
o3d.visualization.draw_geometries([skelelines, tube])