import open3d as o3d
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
from collections import defaultdict

import helpers

pcd = o3d.io.read_point_cloud(r"./cropped_downsampled.pcd")

# for k in range(6) :
#     pcd = pcd.uniform_down_sample(2)
# print(np.asarray(pcd.points).shape)

pcd_octree = o3d.geometry.Octree(max_depth = 3)
pcd_octree.convert_from_point_cloud(pcd)

leaf_nodes = [] # [geom_center, size, origin, density_center, points]
leaf_node_centers = []

# traversal of the octree
def f_traverse(node, node_info):
    early_stop = False
    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1
            #print("{}{}: Internal node at depth {} has {} children and {} points ({})".format('    ' * node_info.depth, node_info.child_index, node_info.depth, n, len(node.indices), node_info.origin))

            # we only want to process nodes / spatial regions with enough points
            early_stop = len(node.indices) < 250
    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            geom_center = node_info.origin + 0.5 * node_info.size
            # let's try a centroid of the points inside the leaf, rather than it's geometric center
            # ok that does not work, lel, we need something else
            density_centroid = np.ones(3)
            comp_x = 0
            comp_y = 0
            comp_z = 0
            pcd_pts = np.asarray(pcd.points)
            for i in node.indices :
                comp_x += pcd_pts[i][0]
                comp_y += pcd_pts[i][1]
                comp_z += pcd_pts[i][2]

            density_centroid[0] = comp_x / len(pcd_pts)
            density_centroid[1] = comp_y / len(pcd_pts)
            density_centroid[2] = comp_z / len(pcd_pts)

            #print("{}{}: Leaf node at depth {} has {} points with center {}".format('    ' * node_info.depth, node_info.child_index,node_info.depth, len(node.indices),geom_center))
            leaf_nodes.append([geom_center, node_info.size, node_info.origin, density_centroid, node.indices])
            leaf_node_centers.append(geom_center)
    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop

pcd_octree.traverse(f_traverse)
N = len(leaf_node_centers)
#o3d.visualization.draw_geometries([helpers.conver_points_to_pcd(leaf_node_centers)])


tree = cKDTree(leaf_node_centers)
# Search radius = a bit larger than the largest leaf diagonal
max_size = max(n[1] for n in leaf_nodes)
radius = max_size * np.sqrt(3) + 1e-6

# indicies in adj and edges correspond to indexes from leaf_node_centers so if there exists a pair (i,j) that forms an edge in the graph than those nodes can be retrieved via those indexes
adj = defaultdict(list)          # adjacency list
edges = []                       # list of (i, j) pairs

for i in range(N):
    # candidates that could possibly touch
    candidates = tree.query_ball_point(leaf_node_centers[i], r=radius)
    oi, si = leaf_nodes[i][0], leaf_nodes[i][1]
    #print(candidates)
    
    
    for j in candidates:
        if j <= i:
            continue
        oj, sj = leaf_nodes[j][0], leaf_nodes[j][1]
        
        # Check face adjacency (touching on a face of positive area)
        # X-direction
        if abs((oi[0] + si) - oj[0]) < 1e-8 or abs((oj[0] + sj) - oi[0]) < 1e-8:
            # Y and Z intervals must overlap with positive length
            if max(oi[1], oj[1]) < min(oi[1]+si, oj[1]+sj) - 1e-8 and \
               max(oi[2], oj[2]) < min(oi[2]+si, oj[2]+sj) - 1e-8:
                adj[i].append(j)
                adj[j].append(i)
                edges.append((i, j))
                continue
        # Y-direction
        if abs((oi[1] + si) - oj[1]) < 1e-8 or abs((oj[1] + sj) - oi[1]) < 1e-8:
            if max(oi[0], oj[0]) < min(oi[0]+si, oj[0]+sj) - 1e-8 and \
               max(oi[2], oj[2]) < min(oi[2]+si, oj[2]+sj) - 1e-8:
                adj[i].append(j)
                adj[j].append(i)
                edges.append((i, j))
                continue
        # Z-direction
        if abs((oi[2] + si) - oj[2]) < 1e-8 or abs((oj[2] + sj) - oi[2]) < 1e-8:
            if max(oi[0], oj[0]) < min(oi[0]+si, oj[0]+sj) - 1e-8 and \
               max(oi[1], oj[1]) < min(oi[1]+si, oj[1]+sj) - 1e-8:
                adj[i].append(j)
                adj[j].append(i)
                edges.append((i, j))
#print(edges)
#print(f"Number of edges: {len(edges)}")
#print(adj)

G = nx.Graph()
G.add_nodes_from(range(N))
G.add_edges_from(edges)

# Attach useful attributes
for i, leaf in enumerate(leaf_nodes):
    G.nodes[i]["center"]  = leaf[0]
    G.nodes[i]["size"]    = leaf[1]
    G.nodes[i]["origin"]  = leaf[2]

# construct a spanning tree/ forest - this is probably not going to be very close to a skeleton
# is again a networkx graph
spanning_tree = nx.minimum_spanning_tree(G)
# print(nx.is_forest(spanning_tree))
# print(G.number_of_edges())
# print(spanning_tree.number_of_edges())
# print(spanning_tree.nodes[5]["center"])

# constructing lineset from the graph with open3d for debugging
pts = []
pts_networkxG = []
for p in range(len(spanning_tree.nodes)) :
    pts.append(spanning_tree.nodes[p]["center"])

for p in range(len(G.nodes)) :
    pts_networkxG.append(spanning_tree.nodes[p]["center"])

lines = [l for l in spanning_tree.edges]
lines_g = [l for l in G.edges]
l_set = o3d.geometry.LineSet(points = o3d.utility.Vector3dVector(pts), lines = o3d.utility.Vector2iVector(lines))
lg_set = o3d.geometry.LineSet(points = o3d.utility.Vector3dVector(pts_networkxG), lines = o3d.utility.Vector2iVector(lines_g))
o3d.io.write_line_set("./CAMPINO_skel.ply", l_set)

o3d.visualization.draw_geometries([l_set])
