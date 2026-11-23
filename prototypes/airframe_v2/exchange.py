"""Neutral CAD, STL, collision placeholders and explicitly assumed mass data."""
import json
from pathlib import Path

import FreeCAD as App
import MeshPart
import Part

from build import joints, parts, world_shape


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n")


def placement_data(placement):
    x, y, z, w = placement.Rotation.Q
    return {"translation_mm": list(placement.Base), "quaternion_wxyz": [w, x, y, z]}


def export(doc, destination):
    if any(abs(obj.Angle.Value) > 1e-8 for obj in joints(doc)):
        raise ValueError("Export requires neutral angles.")
    doc.recompute()
    invalid = [obj.Name for obj in doc.Objects if "Invalid" in obj.State]
    if invalid:
        raise ValueError(f"Invalid FreeCAD objects: {invalid}")
    destination = Path(destination)
    meshes = destination / "visual"
    meshes.mkdir(parents=True, exist_ok=True)
    physical = [obj for obj in parts(doc) if obj.ExportRole == "physical"]
    world = [world_shape(obj) for obj in physical]
    step = destination / "Airframe-V2.step"
    Part.makeCompound(world).exportStep(str(step))
    reloaded = Part.read(str(step))
    round_trip = {}
    if len(reloaded.Solids) != len(world) or not reloaded.isValid():
        raise ValueError(f"STEP round trip failed: {len(reloaded.Solids)} solids.")
    for obj, original, imported in zip(physical, world, reloaded.Solids):
        before, after = original.optimalBoundingBox(), imported.optimalBoundingBox()
        error = max(abs(getattr(before, key) - getattr(after, key))
                    for key in ("XMin", "YMin", "ZMin", "XMax", "YMax", "ZMax"))
        volume_delta = abs(original.Volume - imported.Volume) / original.Volume
        round_trip[obj.Name] = {"bounds_delta_mm": error, "relative_volume_delta": volume_delta}
        if error > 1e-4 or volume_delta > 0.01:
            raise ValueError(f"STEP round trip outside tolerance for {obj.Name}: {round_trip[obj.Name]}")

    frames = {}
    for frame in (obj for obj in doc.Objects if obj.TypeId == "App::Part"):
        owner = frame.getParentGeoFeatureGroup()
        data = {"parent": owner.Name if owner else None,
                "parent_from_frame": placement_data(frame.Placement),
                "world_from_frame_neutral": placement_data(frame.getGlobalPlacement())}
        if "JointAxis" in frame.PropertiesList:
            data["axis_local"] = list(frame.JointAxis)
            data["sample_range_deg"] = [-frame.DisplayLimit.Value, frame.DisplayLimit.Value]
        frames[frame.Name] = data
    manifest = {"up_axis": "+Z", "nose_axis": "+Z", "mm_to_m": 0.001,
                "frames": frames, "parts": {}}
    proxies = {}
    inertials = {}
    density = 1000.0
    for obj in physical:
        local = obj.Shape.copy()
        local.Placement = App.Placement()
        local = local.Solids[0]
        mesh = MeshPart.meshFromShape(Shape=local, LinearDeflection=0.15,
                                      AngularDeflection=0.15, Relative=False)
        if not mesh.isSolid():
            raise ValueError(f"Visual mesh is not closed: {obj.Name}")
        mesh.write(str(meshes / f"{obj.Name}.stl"))
        parent = obj.getParentGeoFeatureGroup()
        manifest["parts"][obj.Name] = {
            "parent": parent.Name, "parent_from_part": placement_data(obj.Placement),
            "world_from_part_neutral": placement_data(obj.getGlobalPlacement()),
            "visual_stl": f"visual/{obj.Name}.stl", "triangles": mesh.CountFacets,
        }
        bounds = local.optimalBoundingBox()
        proxies[obj.Name] = {"type": "box", "center_mm": list(bounds.Center),
                             "half_size_mm": [bounds.XLength / 2, bounds.YLength / 2,
                                              bounds.ZLength / 2]}
        matrix = local.MatrixOfInertia
        inertials[obj.Name] = {
            "volume_mm3": local.Volume, "mass_kg": local.Volume * density * 1e-9,
            "com_mm": list(local.CenterOfMass),
            "inertia_about_com_kg_m2": [
                [matrix.A[4 * i + j] * density * 1e-15 for j in range(3)] for i in range(3)
            ],
        }
    dump(destination / "assembly.json", manifest)
    dump(destination / "collision.json", {
        "status": "external broad-phase placeholders in part-local coordinates",
        "parts": proxies,
    })
    dump(destination / "mass.json", {
        "status": "illustrative homogeneous density, part-local values",
        "uniform_density_kg_m3": density, "parts": inertials,
    })
    return {"step_solids": len(reloaded.Solids), "step_round_trip_by_part": round_trip,
            "closed_visual_meshes": len(physical)}
