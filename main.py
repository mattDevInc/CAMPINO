import open3d as o3d
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
from collections import defaultdict

import helpers

def imports () :
    pcd = o3d.io.read_point_cloud("./cropped_downsampled.pcd")
    return pcd

def ftraverse_verbose (node, node_info) :
    early_stop = False
    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1
            print("{}{}: Internal node at depth {} has {} children and {} points ({}), size={}".format('    ' * node_info.depth, node_info.child_index, node_info.depth, n, len(node.indices), node_info.origin, node_info.size))
            early_stop = len(node.indices) < 250
    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            print("{}{}: Leaf node at depth {} has {} points with origin {}".format('    ' * node_info.depth, node_info.child_index,node_info.depth, len(node.indices), node_info.origin))
    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop

def extract_centers (octree_) :
    leaf_centers= []

    def ftraverse_extract (node, node_info) :
        early_stop = False
        if isinstance(node, o3d.geometry.OctreeInternalNode):
            if isinstance(node, o3d.geometry.OctreeInternalPointNode):
                early_stop = len(node.indices) < 250
        elif isinstance(node, o3d.geometry.OctreeLeafNode):
            if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
                # depth = node_info.depth
                # parent_center = oct_parent_centers[f"{depth - 1}"]
                leaf_center = node_info.origin + 0.5 * node_info.size
                leaf_centers.append((leaf_center, node_info.origin, node_info.size))
        else:
            raise NotImplementedError('Node type not recognized!')
        
        return early_stop
        
    octree_.traverse(ftraverse_extract)
    return leaf_centers

# def dual_graph (leaves) :
#     centers = [n[0] for n in leaves]

#     leaf_tree = cKDTree(centers)
#     # size always stays the same in this situation
#     search_radious = leaves[0][2]

#     adjecent_leaves =  defaultdict(list)
#     edges = []

#     for i in range(len(centers)) :
#         possible_match = leaf_tree.query_ball_point(centers[i], r=search_radious)
#         currently_eval_leaf = centers[i]
#         #print(candidates)


#         for j in possible_match:
#             if j <= i:
#                 continue
#             candidate = centers[j]
            
#             if abs((currently_eval_leaf[0] + search_radious) - candidate[0]) < 1e-8 or abs((candidate[0] + search_radious) - currently_eval_leaf[0]) < 1e-8:
#                 if max(currently_eval_leaf[1], candidate[1]) < min(currently_eval_leaf[1]+search_radious, candidate[1]+search_radious) - 1e-8 and max(currently_eval_leaf[2], candidate[2]) < min(currently_eval_leaf[2]+search_radious, candidate[2]+search_radious) - 1e-8:
#                     adjecent_leaves[i].append(j)
#                     adjecent_leaves[j].append(i)
#                     edges.append((i, j))
#                     continue
#             if abs((currently_eval_leaf[1] + search_radious) - candidate[1]) < 1e-8 or abs((candidate[1] + search_radious) - currently_eval_leaf[1]) < 1e-8:
#                 if max(currently_eval_leaf[0], candidate[0]) < min(currently_eval_leaf[0]+search_radious, candidate[0]+search_radious) - 1e-8 and max(currently_eval_leaf[2], candidate[2]) < min(currently_eval_leaf[2]+search_radious, candidate[2]+search_radious) - 1e-8:
#                     adjecent_leaves[i].append(j)
#                     adjecent_leaves[j].append(i)
#                     edges.append((i, j))
#                     continue
#             if abs((currently_eval_leaf[2] + search_radious) - candidate[2]) < 1e-8 or abs((candidate[2] + search_radious) - currently_eval_leaf[2]) < 1e-8:
#                 if max(currently_eval_leaf[0], candidate[0]) < min(currently_eval_leaf[0]+search_radious, candidate[0]+search_radious) - 1e-8 and max(currently_eval_leaf[1], candidate[1]) < min(currently_eval_leaf[1]+search_radious, candidate[1]+search_radious) - 1e-8:
#                     adjecent_leaves[i].append(j)
#                     adjecent_leaves[j].append(i)
#                     edges.append((i, j))
#         #print(edges)
#         #print(f"Number of edges: {len(edges)}")
#         #print(adj)

#     dual_g = nx.Graph()
#     dual_g.add_nodes_from(range(len(centers)))
#     dual_g.add_edges_from(edges)

#     for i in range(len(centers)):
#         dual_g.nodes[i]["center"]  = centers[i]
#         dual_g.nodes[i]["size"]    = centers[i]

#     return dual_g

def dual_graph (l) :
    N = len(l)
    #o3d.visualization.draw_geometries([helpers.conver_points_to_pcd(leaf_node_centers)])

    centers = [c[0] for c in l]
    tree = cKDTree(centers)
    size = l[0][2]
    # Search radius = a bit larger than the largest leaf diagonal
    max_size = max(n[2] for n in l)
    radius = max_size * np.sqrt(3) + 1e-6

    # indicies in adj and edges correspond to indexes from leaf_node_centers so if there exists a pair (i,j) that forms an edge in the graph than those nodes can be retrieved via those indexes
    adj = defaultdict(list)          # adjacency list
    edges = []                       # list of (i, j) pairs

    for i in range(N):
        # candidates that could possibly touch
        candidates = tree.query_ball_point(centers[i], r=radius)
        oi, si = centers[i], size
        #print(candidates)
        
        
        for j in candidates:
            if j <= i:
                continue
            oj, sj = centers[j], size
            
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

    dual_g = nx.Graph()
    dual_g.add_nodes_from(range(N))
    dual_g.add_edges_from(edges)

    # Attach useful attributes
    for i, leaf in enumerate(l):
        dual_g.nodes[i]["center"] = leaf[0]
        dual_g.nodes[i]["origin"] = leaf[1]
        dual_g.nodes[i]["size"] = leaf[2]

    return dual_g


def main () :
    pcd = imports()
    pcd_octree = o3d.geometry.Octree(max_depth = 5)
    pcd_octree.convert_from_point_cloud(pcd)
    leaf_nodes = extract_centers(pcd_octree)
    pcd_octree.traverse(ftraverse_verbose)
    dual_g_ = dual_graph(leaf_nodes)
    debug_graph_lines = helpers.line_set_from_graph(dual_g_)
    #o3d.visualization.draw_geometries([debug_graph_lines])
    pts_only = helpers.point_only_graph_visualization(dual_g_)
    #o3d.visualization.draw_geometries([pts_only, debug_graph_lines])
    dual_g_M_T = helpers.M_T_graph(pts_only, 0.5 * leaf_nodes[0][2])
    debug_M_T = helpers.line_set_from_graph(dual_g_M_T)
    o3d.visualization.draw_geometries([debug_M_T, pts_only])
    np.save("./hornitos_pts.npy", np.asarray(pcd.points))
    
    #leaf_ctr_pts = helpers.convert_points_to_pcd(leaf_nodes)
    # leaf_origins = [n[0] for n in leaf_nodes]
    # leaf_origins_pts = helpers.convert_points_to_pcd(leaf_origins, color_=[1, 0 ,0])
    #o3d.visualization.draw_geometries([leaf_ctr_pts, leaf_origins_pts,pcd_octree])
    #points = helpers.convert_points_to_pcd([np.array([-81.39800262, -0.14700222, -34.07200149]), np.array([-81.39800262 + 59.99198265075684 * 0.5,  -0.14700222 + 59.99198265075684 * 0.5, -34.07200149 + 59.99198265075684 * 0.5]), np.array([-81.39800262 + 59.99198265075684, -0.14700222 + 59.99198265075684, -34.07200149 + 59.99198265075684])])
    #o3d.visualization.draw_geometries([points])
    # https://geidav.wordpress.com/2017/12/02/advanced-octrees-4-finding-neighbor-nodes/

if __name__ == "__main__" :
    main()

