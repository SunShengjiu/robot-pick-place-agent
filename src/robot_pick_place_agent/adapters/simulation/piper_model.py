"""Adapt the official MJCF to the same pinned repository's current URDF.

Original sources are never edited. Controller settings below are simulation
tuning, not motor specifications or parameters approved for a physical PiPER.
"""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).resolve().parents[2] / "assets/piper"
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 9))
HOME = (0.0, 0.8, -0.7, 0.0, 0.3, 0.0)
KP = (180.0, 220.0, 180.0, 50.0, 40.0, 25.0, 600.0, 600.0)
KD = (12.0, 16.0, 12.0, 3.0, 3.0, 2.0, 6.0, 6.0)
FORCE_LIMITS = (25.0, 25.0, 18.0, 8.0, 8.0, 5.0, 10.0, 10.0)
TCP_OFFSET = (0.0, 0.0, 0.125)


def rpy_quaternion(rpy):
    """URDF fixed-axis Rz(yaw) Ry(pitch) Rx(roll), returned as MuJoCo wxyz."""
    r, p, y = (float(v) / 2 for v in rpy.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def numbers(values):
    return " ".join(format(float(v), ".15g") for v in values)


def build_model_xml():
    root = ET.parse(ASSETS / "upstream/piper_description.xml").getroot()
    urdf = ET.parse(ASSETS / "upstream/piper_description.urdf").getroot()
    root.set("model", "piper_m1_current_urdf")
    root.find("compiler").attrib.update({"meshdir": str(ASSETS / "meshes"), "autolimits": "true", "inertiafromgeom": "false"})
    root.remove(root.find("size"))
    ET.SubElement(root, "option", timestep="0.001", gravity="0 0 -9.81", integrator="implicitfast")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "headlight", ambient="0.5 0.5 0.5", diffuse="0.6 0.6 0.6")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "geom", friction="0.7 0.005 0.0001", solref="0.006 1", solimp="0.95 0.99 0.001")
    for mesh in root.findall("asset/mesh"):
        mesh.set("file", Path(mesh.get("file")).name)
    world = root.find("worldbody")
    original_children = list(world)
    base = ET.SubElement(world, "body", name="base_link")
    for child in original_children:
        world.remove(child)
        base.append(child)
    link6 = root.find(".//body[@name='link6']")
    gripper = ET.SubElement(link6, "body", name="gripper_base")
    for child in list(link6):
        if (child.tag == "geom" and child.get("mesh") == "gripper_base") or (child.tag == "body" and child.get("name") in ("link7", "link8")):
            link6.remove(child)
            gripper.append(child)
    for link in urdf.findall("link"):
        body = root.find(f".//body[@name='{link.get('name')}']")
        old = body.find("inertial")
        if old is not None:
            body.remove(old)
        inertial = link.find("inertial")
        inertia = inertial.find("inertia")
        if inertial.find("origin").get("rpy") != "0 0 0":
            raise ValueError("Non-identity URDF inertial frame needs an explicit tensor rotation")
        ET.SubElement(body, "inertial", pos=inertial.find("origin").get("xyz"),
                      mass=inertial.find("mass").get("value"),
                      fullinertia=" ".join(inertia.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
    for joint in urdf.findall("joint"):
        body = root.find(f".//body[@name='{joint.find('child').get('link')}']")
        origin = joint.find("origin")
        body.set("pos", origin.get("xyz"))
        body.set("quat", numbers(rpy_quaternion(origin.get("rpy"))))
        if joint.get("type") == "fixed":
            continue
        local = body.find("joint")
        limit = joint.find("limit")
        local.attrib.update({"axis": joint.find("axis").get("xyz"),
                             "range": f"{limit.get('lower')} {limit.get('upper')}",
                             "damping": "0.05", "armature": "0.005"})
    # Visual triangles are kept intact; mesh contacts use MuJoCo convex hulls.
    for geom in list(root.findall(".//geom[@mesh]")):
        body = next(b for b in root.iter("body") if geom in list(b))
        geom.set("name", f"{body.get('name')}_collision")
        geom.set("group", "3")
        geom.set("rgba", "0.4 0.7 0.4 0.2")
        visible = deepcopy(geom)
        visible.attrib.update({"name": f"{body.get('name')}_visual", "contype": "0", "conaffinity": "0", "group": "1",
                               "rgba": "0.78 0.81 0.85 1" if body.get("name") not in ("link6", "link7", "link8") else "0.20 0.23 0.27 1"})
        body.append(visible)
    ET.SubElement(link6, "site", name="tcp", pos=numbers(TCP_OFFSET), size="0.004", rgba="0.9 0.2 0.1 1", group="2")
    ET.SubElement(world, "geom", name="table", type="plane", pos="0 0 -0.001", size="1 1 0.02", rgba="0.50 0.54 0.57 1")
    ET.SubElement(world, "light", pos="0.3 -0.4 1.8", dir="-0.1 0.1 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "light", pos="-0.8 0.3 1.0", dir="0.5 0 -1", diffuse="0.5 0.5 0.5", castshadow="false")
    contact = ET.SubElement(root, "contact")
    # CAD bearing envelopes overlap by 6 mm at this permanently mated pair.
    ET.SubElement(contact, "exclude", body1="base_link", body2="link1")
    actuators = root.find("actuator")
    actuators.clear()
    for name, force in zip(JOINT_NAMES, FORCE_LIMITS):
        ET.SubElement(actuators, "motor", name=f"{name}_motor", joint=name, gear="1",
                      ctrllimited="true", ctrlrange=numbers((-force, force)))
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")
