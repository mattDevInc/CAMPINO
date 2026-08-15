import open3d as o3d
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
from collections import defaultdict

def convert_points_to_pcd (pts, color_ = None) -> o3d.geometry.PointCloud :
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if color_ != None :
        pcd.paint_uniform_color(color_)
    return pcd

def line_set_from_graph (graph_ : nx.Graph) -> o3d.geometry.LineSet :
    pts = []
    for i in range(len(graph_.nodes)) :
        pts.append(graph_.nodes[i]["center"])

    lines = [l for l in graph_.edges]
    l_set = o3d.geometry.LineSet(points = o3d.utility.Vector3dVector(pts), lines = o3d.utility.Vector2iVector(lines))
    return l_set

def point_only_graph_visualization (graph_ : nx.Graph) :
    pts = []
    # we always end up with an uneven number of nodes
    cols = [[0, 1, 0]] * (len(graph_.nodes) + len(graph_.edges))
    mumbo_jumbo = 0
    visted = set()

    graph_edges = [e for e in graph_.edges]
    for p in range(len(graph_edges)) :
        og_node0 = graph_edges[p][0]
        og_node1 = graph_edges[p][1]

        nd1 = graph_.nodes[og_node0]["center"]
        nd2 = graph_.nodes[og_node1]["center"]
        # calculate midpoint
        midpoint = np.array([(nd1[0] + nd2[0]) / 2, (nd1[1] + nd2[1]) / 2, (nd1[2] + nd2[2]) / 2])
        if og_node0 not in visted :
            pts.append(nd1)
            visted.add(og_node0)
            cols[mumbo_jumbo] = [0, 1, 0]
            mumbo_jumbo += 1
        if og_node1 not in visted :
            pts.append(nd2)
            visted.add(og_node1)
            cols[mumbo_jumbo] = [0, 1, 0]
            mumbo_jumbo += 1
        
        pts.append(midpoint)
        cols[mumbo_jumbo] = [1, 0, 0]
        mumbo_jumbo += 1
   
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(cols)
    return pcd

def M_T_graph (pcd : o3d.geometry.PointCloud, size) -> nx.Graph :
    # just create a graph again but with a vertex in the middle of each edge
    
    #o3d.visualization.draw_geometries([helpers.conver_points_to_pcd(leaf_node_centers)])
    centers = np.asanyarray(pcd.points)
    N = len(centers)
    tree = cKDTree(centers)
    # Search radius = a bit larger than the largest leaf diagonal
    radius = size * np.sqrt(3) + 1e-6

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
    for i in range(len(centers)):
        dual_g.nodes[i]["center"] = centers[i]


    return dual_g