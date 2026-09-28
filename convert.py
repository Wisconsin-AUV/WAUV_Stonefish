# convert OnShape (our PDM) URDF exports to Stonefish XML
# TODO generalize this and add more if needed

import sys
import xml.etree.ElementTree as ET
import numpy as np
import os
import shutil

## CONFIG
SRC_ROOT = "urdf_models"
OUT_DIR = "models"
OUT_MESH_DIR = os.path.join(OUT_DIR, "meshes")
PHYSICS = "submerged"
ROUGHNESS = "0.5"
MATERIAL = "Aluminum"

looks = {}
used_meshes = set()

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

def matrix_to_xyz_rpy(T):
    # turn a 4x4 matrix back into (xyz, rpy) lists
    R = T[:3, :3]
    xyz = T[:3, 3].tolist()
    cos_pitch = np.hypot(R[0, 0], R[1, 0])
    pitch = np.arctan2(-R[2, 0], cos_pitch)
    if cos_pitch > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[2, 0] * R[0, 1], R[1, 1])
        yaw = 0.0
    return xyz, [roll, pitch, yaw]
 
def fmt(values):
    return " ".join(f"{0.0 if abs(v) < 1e-9 else v:.6f}" for v in values) 

def look_name(visual):
    color = visual.find("material/color") # <material><color rgba=".."/></material>
    rgba = color.get("rgba") if color is not None else "0.5 0.5 0.5 1"
    
    if rgba not in looks:
        looks[rgba] = f"look_{len(looks)}"
    return looks[rgba]

def mesh_path(visual):
    mesh_file = os.path.basename(visual.find("geometry/mesh").get("filename"))
    used_meshes.add(mesh_file)
    return os.path.join(OUT_MESH_DIR, mesh_file)
 
def make_link(tag, name, visual):
    # create a Stonefish <link>/<base_link> of type 'model' from one URDF <visual>
    el = ET.Element(tag, name=name, type="model", physics=PHYSICS)

    xyz, rpy = matrix_to_xyz_rpy(origin_to_matrix(visual.find("origin")))

    for section in ("physical", "visual"):
        part = ET.SubElement(el, section)
        ET.SubElement(part, "mesh", filename=mesh_path(visual), scale="1.0")
        ET.SubElement(part, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
    ET.SubElement(el, "material", name=MATERIAL)
    ET.SubElement(el, "look", name=look_name(visual))
    return el
 
def make_joint(name, parent, child, T):
    # create a Stonefish fixed <joint> placing 'child' at transform T in 'parent'
    el = ET.Element("joint", name=name, type="fixed")
    ET.SubElement(el, "parent", name=parent)
    ET.SubElement(el, "child", name=child)
    xyz, rpy = matrix_to_xyz_rpy(T)
    ET.SubElement(el, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
    return el

def add_link(tag, name):
    # add one URDF link. Extra visuals become extra links fixed on top of it
    visuals = links[name].findall("visual")
    new_links.append(make_link(tag, name, visuals[0]))

    for i, extra in enumerate(visuals[1:], start=1):         # any additional visuals
        extra_name = f"{name}__visual{i}"
        new_links.append(make_link("link", extra_name, extra))
        new_joints.append(make_joint(extra_name + "_joint", name, extra_name, np.eye(4)))
 
def is_real(name):
    return links[name].find("visual") is not None
 
def walk(name, real_parent, T_from_real_parent):
    # depth-first walk, empty links are skipped by multiplying their transforms through
    for child, T_joint, joint_name in children_of[name]:
        T = T_from_real_parent @ T_joint

        if is_real(child):
            new_joints.append(make_joint(joint_name, real_parent, child, T))
            add_link("link", child)
            walk(child, child, np.eye(4))
        else:
            walk(child, real_parent, T)
 ##########################################

# get the model name
if len(sys.argv) != 2:
    raise RuntimeError("Plz specify model name")

model_name = sys.argv[1]
model_dir = os.path.join(SRC_ROOT, model_name)
SRC_MESH_DIR = os.path.join(model_dir, "meshes")
fname = os.path.join(model_dir, "urdf", model_name + ".urdf")

if not os.path.exists(fname):
    raise FileNotFoundError(f"No URDF at {fname}")

# TODO probably wanna add a bit more functionality

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

add_link("base_link", root_name)
walk(root_name, root_name, np.eye(4))  

# assemble everything
for el in new_links + new_joints:
    robot.append(el)
ET.SubElement(robot, "world_transform", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

# finally start exporting

os.makedirs(OUT_MESH_DIR, exist_ok=True)

base = os.path.join(OUT_DIR, model_name)

scenario = ET.Element("scenario")

looks_root = ET.SubElement(scenario, "looks")

for rgba, lname in looks.items():
    r, g, b = rgba.split()[:3]
    ET.SubElement(looks_root, "look", name=lname, rgb=f"{r} {g} {b}", roughness=ROUGHNESS)

scenario.append(robot)

out_tree = ET.ElementTree(scenario)

ET.indent(out_tree)

out_tree.write(base + ".scn", encoding="utf-8", xml_declaration=True)

# copy only the meshes the robot uses
for mesh_file in sorted(used_meshes):
    src = os.path.join(SRC_MESH_DIR, mesh_file)
    if not os.path.exists(src):
        raise FileNotFoundError(f"Mesh referenced in URDF not found: {src}")
    shutil.copy2(src, os.path.join(OUT_MESH_DIR, mesh_file)) 