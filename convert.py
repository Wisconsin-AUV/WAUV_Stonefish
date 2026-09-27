# convert OnShape (our PDM) URDF exports to Stonefish XML
# TODO generalize this and add more if needed

import sys
import xml.etree.ElementTree as ET
import numpy as np
import os

####### funcs ##########################
def make_transform(xyz, rpy):
    # Build a 4x4 homogeneous transform from a position and roll/pitch/yaw.
    r, p, y = rpy
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
    
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0, 0, 1]])
    
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = xyz
    return T
 
def origin_to_matrix(origin_el):
    if origin_el is None: # this means "no offset"
        return np.eye(4)
    xyz = [float(v) for v in origin_el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin_el.get("rpy", "0 0 0").split()]
    return make_transform(xyz, rpy)
 ##########################################

# get the filename
if len(sys.argv != 2):
    raise RuntimeError("Plz specify URDF directory")
else:
    fname = sys.argv[1]

# TODO probably wanna add a bit more functionality

# required Stonefish config
MESH_DIR = "meshes"
MATERIAL = "Aluminium"
PHYSICS = "submerged"
ROUGHNESS = "0.5"

# read the URDF tree
og_tree = ET.parse(fname)

# get the root
og_root = og_tree.getroot()

# OnShape exports all these joints and links in ALPHABETIC order
# sort through them before writing anything
# save all links and track their children
links = {link.get("name"): link for link in og_root.findall("link")}
children_of = {name: [] for name in links}
child_names = set()

for joint in og_root.findall("joint"):
    parent = joint.find("parent").get("link")
    child = joint.find("child").get("link")
    T = origin_to_matrix(joint.find("origin"))
    children_of[parent].append((child, T, joint.get("name")))
    child_names.add(child)

# find the root as the one link that is never a child
root = [name for name in links if name not in child_names]

root_name = root[0]

# create the root for the new Stonefish XML tree
robot = ET.Element("robot", name=og_root.get("name", "Robot"), fixed="false", self_collisions="false")
 
new_links = [] # Stonefish <link> elements, in order
new_joints = [] # Stonefish <joint> elements, parents first

