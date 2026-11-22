"""Run inside FreeCAD's bundled Python to inspect a disposable CAD copy."""

import argparse
import json
import sys
from pathlib import Path


def snapshot(doc):
    doc.recompute()
    invalid = [obj.Name for obj in doc.Objects if "Invalid" in obj.State]
    if invalid:
        raise RuntimeError(f"Invalid CAD features: {invalid}")
    parts = {}
    joints = {}

    def visit(group, owner):
        for obj in group.Group:
            if obj.TypeId == "App::Part":
                if "JointAxis" in obj.PropertiesList:
                    joints[obj.Name] = {"parent": owner.Name, "axis": list(obj.JointAxis),
                                        "travel_deg": obj.Travel.Value}
                    owner_for_child = obj
                else:
                    owner_for_child = owner
                visit(obj, owner_for_child)
            elif "Density" in obj.PropertiesList and obj.Density > 0:
                shape = obj.Shape.copy()
                if shape.isNull() or not shape.isValid() or len(shape.Solids) != 1:
                    raise RuntimeError(f"{obj.Name} is not a valid single solid.")
                shape.Placement = group.getGlobalPlacement() * shape.Placement
                box = shape.optimalBoundingBox(False, False)
                parts[obj.Name] = {"parent": owner.Name, "volume_mm3": shape.Volume,
                                   "bounds_mm": [[box.XMin, box.YMin, box.ZMin],
                                                 [box.XMax, box.YMax, box.ZMax]]}

    visit(doc.Vehicle, doc.Vehicle)
    poses = []
    for sign in (0, 1, -1):
        commands = {}
        for i, (name, joint) in enumerate(joints.items()):
            angle = sign * (-1)**i * 0.8 * joint["travel_deg"]
            doc.getObject(name).Angle = angle
            commands[name] = angle
        doc.recompute()
        world = {}
        for name in [doc.Vehicle.Name, *joints]:
            pose = doc.getObject(name).getGlobalPlacement()
            x, y, z, w = pose.Rotation.Q
            world[name] = {"position_mm": list(pose.Base), "quat_wxyz": [w, x, y, z]}
        poses.append({"commands_deg": commands, "world": world})
    for name in joints:
        doc.getObject(name).Angle = 0
    doc.recompute()
    return {"parts": parts, "joints": joints, "poses": poses}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("edited", type=Path)
    parser.add_argument("--library", type=Path, required=True)
    args = parser.parse_args()
    sys.path[:0] = [str(args.library), str(Path(__file__).resolve().parents[1])]
    import FreeCAD as app
    from export_sim import export

    doc = app.openDocument(str(args.baseline))
    baseline = snapshot(doc)
    sheet = doc.Parameters
    cell = sheet.getCellFromAlias("FinSpan")
    original = sheet.FinSpan.getValueAs("mm").Value
    baseline_dims = {row: sheet.getContents(f"B{row}") for row in range(2, 15)}
    replacement = f"{original + 4:g} mm"
    sheet.set(cell, replacement)
    doc.recompute()
    args.edited.mkdir()
    edited_path = args.edited / args.baseline.name
    doc.saveAs(str(edited_path))
    app.closeDocument(doc.Name)
    doc = app.openDocument(str(edited_path))
    edited = snapshot(doc)
    edited_dims = {row: doc.Parameters.getContents(f"B{row}") for row in baseline_dims}
    changed = [f"B{row}" for row in baseline_dims if baseline_dims[row] != edited_dims[row]]
    if changed != [cell] or doc.Parameters.FinSpan.getValueAs("mm").Value != original + 4:
        raise RuntimeError(f"Expected one persistent parameter edit, got {changed}.")
    export(doc, args.edited)
    app.closeDocument(doc.Name)

    report = {"freecad": ".".join(app.Version()[:3]),
              "edit": {"alias": "FinSpan", "cell": cell, "before": f"{original:g} mm",
                       "after": replacement, "changed_cells": changed},
              "baseline": baseline, "edited": edited}
    (args.edited / "cad_probe.json").write_text(json.dumps(report))


if __name__ == "__main__":
    main()
