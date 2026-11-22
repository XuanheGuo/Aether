"""Export the active neutral FreeCAD assembly as body-local meshes and MJCF."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import FreeCAD as App
import MeshPart

V = App.Vector


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def meters(vector):
    return numbers(v / 1000 for v in vector)


def quaternion(rotation):
    x, y, z, w = rotation.Q
    return numbers((w, x, y, z))


def inertial(body, pieces):
    """Aggregate CAD volume inertias with each part's concept density."""
    masses = [shape.Volume * obj.Density * 1e-9 for obj, shape in pieces]
    mass = sum(masses)
    center = sum((shape.CenterOfMass * m for (_, shape), m in zip(pieces, masses)),
                 V()) / mass
    tensor = []
    for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)):
        entry = 0.0
        for (obj, shape), m in zip(pieces, masses):
            d = (shape.CenterOfMass - center) / 1000
            entry += shape.MatrixOfInertia.A[4*i+j] * obj.Density * 1e-15
            entry += m * ((d.dot(d) if i == j else 0) - d[i]*d[j])
        tensor.append(entry)
    ET.SubElement(body, "inertial", pos=meters(center), mass=numbers((mass,)),
                  fullinertia=numbers(tensor))


def export(doc, destination):
    destination = Path(destination)
    meshes = destination / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    doc.recompute()
    assembly = doc.Vehicle
    frames = [obj for obj in doc.Objects if "JointAxis" in obj.PropertiesList]
    if any(abs(frame.Angle.Value) > 1e-7 for frame in frames):
        raise ValueError("Reset all CAD hinge Angles to zero before exporting.")
    if any("Invalid" in obj.State for obj in doc.Objects):
        raise ValueError("The FreeCAD document contains an invalid feature.")

    model = ET.Element("mujoco", model="Aether V1")
    ET.SubElement(model, "compiler", angle="degree", meshdir="meshes",
                  inertiafromgeom="false", autolimits="true")
    ET.SubElement(model, "option", timestep="0.002", integrator="implicitfast")
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", azimuth="134", elevation="-9")
    ET.SubElement(visual, "rgba", haze="0.16 0.19 0.23 1")
    ET.SubElement(visual, "scale", forcewidth="0.02", jointlength="0.10")
    defaults = ET.SubElement(model, "default")
    surface = ET.SubElement(defaults, "default", {"class": "surface"})
    ET.SubElement(surface, "geom", type="mesh", group="0", contype="0",
                  conaffinity="0", mass="0")
    contact = ET.SubElement(defaults, "default", {"class": "contact"})
    ET.SubElement(contact, "geom", group="3", contype="2", conaffinity="1",
                  rgba="0.8 0.2 0.2 0.25", mass="0")
    assets = ET.SubElement(model, "asset")
    ET.SubElement(assets, "texture", name="floor", type="2d", builtin="checker",
                  rgb1="0.16 0.20 0.23", rgb2="0.19 0.24 0.27", width="256", height="256")
    ET.SubElement(assets, "material", name="deck", texture="floor",
                  texrepeat="6 6", texuniform="true", reflectance="0.10")
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", directional="true", pos="0 -2 3",
                  dir="0 1 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "light", directional="true", pos="2 2 1",
                  dir="-1 -1 -0.3", diffuse="0.35 0.4 0.48", castshadow="false")
    ET.SubElement(world, "geom", name="floor", type="plane", size="5 5 0.1",
                  material="deck", contype="1", conaffinity="2")
    root = ET.SubElement(world, "body", name=assembly.Name)
    ET.SubElement(root, "freejoint", name="Base")
    motors = ET.SubElement(model, "actuator")
    manifest = {"joints": {}, "parts": {}}

    def visit(container, owner, body, pieces):
        for obj in container.Group:
            if obj.TypeId == "App::Part":
                if "JointAxis" not in obj.PropertiesList:
                    visit(obj, owner, body, pieces)
                    continue
                placement = owner.getGlobalPlacement().inverse() * obj.getGlobalPlacement()
                child = ET.SubElement(body, "body", name=obj.Name,
                                      pos=meters(placement.Base), quat=quaternion(placement.Rotation))
                travel = obj.Travel.Value
                ET.SubElement(child, "joint", name=obj.Name, type="hinge",
                              axis=numbers(obj.JointAxis), range=numbers((-travel, travel)),
                              damping="0.008", armature="0.00002")
                ET.SubElement(motors, "position", name=obj.Name + "_target",
                              joint=obj.Name, kp="0.6", kv="0.018",
                              ctrlrange=numbers((-math.radians(travel), math.radians(travel))),
                              forcerange="-0.25 0.25")
                manifest["joints"][obj.Name] = {
                    "parent": owner.Name, "axis": list(obj.JointAxis),
                    "parent_position_mm": list(placement.Base),
                    "parent_quaternion_wxyz": quaternion(placement.Rotation),
                    "range_degrees": [-travel, travel],
                }
                child_pieces = []
                visit(obj, obj, child, child_pieces)
                inertial(child, child_pieces)
                continue
            if "Density" not in obj.PropertiesList or obj.Density <= 0:
                continue
            shape = obj.Shape.copy()
            if shape.isNull() or not shape.isValid() or len(shape.Solids) != 1 or shape.Volume <= 0:
                raise ValueError(f"{obj.Name} did not recompute to one valid solid.")
            placement = owner.getGlobalPlacement().inverse() * container.getGlobalPlacement()
            shape.Placement = placement * shape.Placement
            pieces.append((obj, shape.Solids[0]))
            mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.24,
                                          AngularDeflection=0.13, Relative=False)
            path = meshes / f"{obj.Name}.stl"
            mesh.write(str(path))
            ET.SubElement(assets, "mesh", name=obj.Name, file=path.name, scale=".001 .001 .001",
                          inertia="convex")
            ET.SubElement(body, "geom", {"class": "surface", "name": obj.Name,
                                         "mesh": obj.Name, "rgba": obj.SimulationRGB + " 1"})
            manifest["parts"][obj.Name] = {
                "body": owner.Name, "volume_mm3": round(shape.Volume, 3),
                "mesh_facets": mesh.CountFacets,
            }
            proxy = obj.ContactProxy
            if proxy:
                box = obj.Shape.BoundBox
                extents = (box.XLength, box.YLength, box.ZLength)
                size = (max(extents[:2]), extents[2]) if proxy == "cylinder" else extents
                ET.SubElement(body, "geom", {"class": "contact", "name": obj.Name + "_contact",
                                            "type": proxy, "pos": meters(placement.multVec(box.Center)),
                                            "quat": quaternion(placement.Rotation),
                                            "size": numbers(v / 2000 for v in size)})
    root_pieces = []
    visit(assembly, assembly, root, root_pieces)
    inertial(root, root_pieces)
    ET.indent(model)
    ET.ElementTree(model).write(destination / "aether_v1.xml", encoding="utf-8", xml_declaration=True)
    (destination / "assembly.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest
