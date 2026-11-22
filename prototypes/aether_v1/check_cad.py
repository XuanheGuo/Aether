"""Exercise recompute, joint poses and clearances on a saved Aether document."""

import json
from itertools import product
from pathlib import Path

import FreeCAD as App


def check(path):
    doc = App.openDocument(str(path))
    pieces = [obj for obj in doc.Objects if "Density" in obj.PropertiesList and obj.Density > 0]

    def volumes():
        doc.recompute()
        errors = [obj.Name for obj in doc.Objects if "Invalid" in obj.State]
        assert not errors, errors
        assert all(obj.Shape.isValid() and len(obj.Shape.Solids) == 1 for obj in pieces)
        return {obj.Name: obj.Shape.Volume for obj in pieces}

    baseline = volumes()
    sheet = doc.Parameters
    for cell, value in {"B2": "244 mm", "B3": "1110 mm", "B4": "6 mm",
                        "B5": "365 mm", "B9": "142 mm"}.items():
        sheet.set(cell, value)
    edited = volumes()
    for part in ("MainShell", "NoseShell", "TailShroud", "VaneSkin1"):
        assert abs(baseline[part] - edited[part]) > 1, part

    # Reopen the original so edits above never touch the deliverable.
    App.closeDocument(doc.Name)
    doc = App.openDocument(str(path))
    joints = [doc.getObject(obj) for obj in ("Vane1", "Vane2", "Vane3", "Vane4", "GimbalX", "GimbalY")]

    def record(angles):
        for obj, angle in zip(joints, angles):
            obj.Angle = angle
        doc.recompute()
        output = {}
        for obj, angle in zip(joints, angles):
            expected = App.Rotation(obj.JointAxis, angle).Q
            actual = obj.Placement.Rotation.Q
            assert abs(sum(a*b for a, b in zip(expected, actual))) > 1 - 1e-10, (
                obj.Name, angle, actual, expected)
            pose = obj.getGlobalPlacement()
            x, y, z, w = pose.Rotation.Q
            output[obj.Name] = {"position_mm": list(pose.Base), "quaternion_wxyz": [w, x, y, z]}
        return {"angles_degrees": dict(zip((obj.Name for obj in joints), angles)), "global": output}

    limits = [obj.Travel.Value for obj in joints]
    poses = [record([0] * 6), record(limits), record([-v for v in limits]),
             record([v * sign for v, sign in zip(limits, (1, -1, 1, -1, -1, 1))])]

    def world_shape(obj):
        shape = obj.Shape.copy()
        shape.Placement = obj.getGlobalPlacement()
        return shape

    worst_overlap = 0.0
    nozzle_floor = float("inf")
    sweep_count = 0
    for x, y in product((-limits[4], -limits[4]/2, 0, limits[4]/2, limits[4]),
                        (-limits[5], -limits[5]/2, 0, limits[5]/2, limits[5])):
        doc.GimbalX.Angle = x
        doc.GimbalY.Angle = y
        doc.recompute()
        nozzle = world_shape(doc.NozzleBell)
        ring = world_shape(doc.GimbalRing)
        shroud = world_shape(doc.TailShroud)
        nozzle_floor = min(nozzle_floor, nozzle.BoundBox.ZMin)
        for a, b in ((nozzle, ring), (nozzle, shroud), (ring, shroud)):
            worst_overlap = max(worst_overlap, abs(a.common(b).Volume))
        sweep_count += 1
    vane_overlap = 0.0
    for scale in (-1, -.5, 0, .5, 1):
        record([scale * limit for limit in limits[:4]] + [0, 0])
        hull = world_shape(doc.MainShell)
        for i in range(1, 5):
            vane = world_shape(doc.getObject(f"VaneSkin{i}"))
            saddle = world_shape(doc.getObject(f"Saddle{i}"))
            vane_overlap = max(vane_overlap, abs(vane.common(hull).Volume),
                               abs(vane.common(saddle).Volume))
    assert worst_overlap < 0.1 and vane_overlap < 0.1 and nozzle_floor > 0, (
        worst_overlap, vane_overlap, nozzle_floor)
    report = {"freecad": ".".join(App.Version()[:3]), "valid_solids": len(baseline),
              "parameter_edit_recomputed": True, "poses": poses,
              "gimbal_sweep_points": sweep_count, "vane_sweep_points_per_vane": 5,
              "max_tail_overlap_mm3": worst_overlap, "max_vane_overlap_mm3": vane_overlap,
              "min_nozzle_bounding_box_floor_clearance_mm": round(nozzle_floor, 3)}
    path.with_name("cad_checks.json").write_text(json.dumps(report, indent=2) + "\n")
    App.closeDocument(doc.Name)
    return report
