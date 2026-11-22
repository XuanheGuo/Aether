"""A render-only Iris clone and native MuJoCo trajectory decorations."""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from prototypes.iris_closed_loop.simulator import MODEL

ASSETS = Path(__file__).with_name("assets")
IDENTITY = np.eye(3).ravel()
VISUAL_GROUP = "2"
COLLISION_GROUP = 3
PATH_COLOR = np.array([.10, .73, .94, .86])
AXES = (
    ([.7, 0., .012], [1., .35, .25, 1.]),
    ([0., .7, .012], [.4, .96, .53, 1.]),
    ([0., 0., .7], [.34, .70, 1., 1.]),
)


def display_model():
    root = ET.parse(MODEL).getroot()
    root.set("model", "Aether · Iris")
    ET.SubElement(root, "statistic", center="0 0 1.15", extent="3")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", ambient=".32 .35 .42", diffuse=".77 .76 .74")
    assets = ET.SubElement(root, "asset")
    ET.SubElement(assets, "texture", name="ground", type="2d", builtin="checker",
                  width="256", height="256", rgb1=".19 .23 .28", rgb2=".26 .31 .36")
    ET.SubElement(assets, "material", name="ground", texture="ground",
                  texrepeat="18 18", texuniform="true", reflectance="0")
    for name in ("iris.stl", "iris_prop_ccw.obj", "iris_prop_cw.obj"):
        ET.SubElement(assets, "mesh", name=name, file=name)
    world = root.find("worldbody")
    world.find("geom[@name='floor']").set("material", "ground")
    iris = world.find("body[@name='iris']")
    iris.find("geom[@name='base_contact']").set("group", str(COLLISION_GROUP))
    ET.SubElement(iris, "geom", name="fuselage_visual", type="mesh", mesh="iris.stl",
                  mass="0", contype="0", conaffinity="0", group=VISUAL_GROUP, rgba=".85 .87 .89 1")
    for i in range(4):
        rotor = iris.find(f"body[@name='rotor_{i}']")
        mesh = "iris_prop_ccw.obj" if i < 2 else "iris_prop_cw.obj"
        rgba = ".30 .63 .89 1" if i in (0, 2) else ".17 .24 .33 1"
        ET.SubElement(rotor, "geom", name=f"prop_{i}_visual", type="mesh", mesh=mesh,
                      mass="0", contype="0", conaffinity="0", group=VISUAL_GROUP, rgba=rgba)
    files = {file.name: file.read_bytes() for file in ASSETS.iterdir()
             if file.suffix in {".stl", ".obj"}}
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), files)


def decorate(scene, points):
    scene.ngeom = 0

    def line(start, end, rgba, width):
        if scene.ngeom == scene.maxgeom or np.linalg.norm(np.asarray(end) - start) < 1e-6:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                            np.ones(3), np.zeros(3), IDENTITY, rgba)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, start, end)
        scene.ngeom += 1

    for end, color in AXES:
        line([0., 0., .012], end, color, .011)
    for start, end in zip(points, points[1:], strict=False):
        line(start, end, PATH_COLOR, .011)


def attitude(quat):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, quat)
    matrix = matrix.reshape(3, 3)
    return np.rad2deg([
        np.arctan2(matrix[2, 1], matrix[2, 2]),
        np.arcsin(np.clip(-matrix[2, 0], -1, 1)),
        np.arctan2(matrix[1, 0], matrix[0, 0]),
    ])
