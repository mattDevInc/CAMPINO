import open3d as o3d
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
from collections import defaultdict

import helpers

def imports () :
    pcd = o3d.io.read_point_cloud("./cropped.pcd")
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

def extract_octree_leaves (octree_) :
    leafes = []

    def ftraverse_extract (node, node_info) :
        early_stop = False
        if isinstance(node, o3d.geometry.OctreeInternalNode):
            if isinstance(node, o3d.geometry.OctreeInternalPointNode):
                early_stop = len(node.indices) < 250
        elif isinstance(node, o3d.geometry.OctreeLeafNode):
            if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
                leafes.append((node_info.origin, node_info.size))
        else:
            raise NotImplementedError('Node type not recognized!')
        
    octree_.traverse(ftraverse_extract)
    return leafes

def extract_octree_structure (octree_) :
    oct_struct = dict()
    

    def ftraverse_extract (node, node_info) :
        early_stop = False
        if isinstance(node, o3d.geometry.OctreeInternalNode):
            if isinstance(node, o3d.geometry.OctreeInternalPointNode):
                #oct_sctruct[]
                early_stop = len(node.indices) < 250
        elif isinstance(node, o3d.geometry.OctreeLeafNode):
            if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
                #leafes.append((node_info.origin, node_info.size))
                raise NotImplementedError('not implemented')
        else:
            raise NotImplementedError('Node type not recognized!')
        
    octree_.traverse(ftraverse_extract)



def main () :
    pcd = imports()
    pcd_octree = o3d.geometry.Octree(max_depth = 2)
    pcd_octree.convert_from_point_cloud(pcd)
    leaf_nodes = extract_octree_leaves(pcd_octree)
    extract_octree_structure(pcd_octree)

if __name__ == "__main__" :
    main()

