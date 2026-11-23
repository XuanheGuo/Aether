"""Build, capture and verify into a new output directory."""
import json
from pathlib import Path

import FreeCAD as App

from build import ROOT, build, parts, visibility, world_shape
from exchange import dump, export
from verify import valid_shapes, verify


def save_reload(doc, path):
    doc.recompute()
    doc.saveAs(str(path))
    App.closeDocument(doc.Name)
    loaded = App.openDocument(str(path))
    valid_shapes(loaded)
    return loaded


def run(destination):
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    baseline_dir = destination / "model"
    baseline_dir.mkdir()
    baseline = build()
    if App.GuiUp:
        from views import render
        render(baseline, destination / "views")
    visibility(baseline)
    baseline_path = baseline_dir / "Airframe-V2.FCStd"
    baseline = save_reload(baseline, baseline_path)
    checks = verify(baseline)
    checks["exchange"] = export(baseline, baseline_dir)
    baseline_shapes = {obj.Name: (world_shape(obj).Volume, world_shape(obj).optimalBoundingBox())
                       for obj in parts(baseline)}
    App.closeDocument(baseline.Name)

    variant_patch = json.loads((ROOT / "parameters/longer-nose.json").read_text())
    variant_dir = destination / "variant-longer-nose"
    variant_dir.mkdir()
    variant = App.openDocument(str(baseline_path))
    table = variant.Parameters
    baseline_parameters = json.loads((ROOT / "parameters/baseline.json").read_text())
    for row, name in enumerate(baseline_parameters, 2):
        if name in variant_patch:
            table.set(f"B{row}", variant_patch[name])
    variant.Label = "Airframe-V2 · Longer nose"
    visibility(variant)
    variant_path = variant_dir / "Airframe-V2-longer-nose.FCStd"
    variant = save_reload(variant, variant_path)
    variant_checks = verify(variant)
    variant_exchange = export(variant, variant_dir)
    variant_step = variant_dir / "Airframe-V2.step"
    variant_step.rename(variant_dir / "Airframe-V2-longer-nose.step")
    changes = {}
    for obj in parts(variant):
        shape = world_shape(obj)
        old_volume, old_bounds = baseline_shapes[obj.Name]
        new_z = shape.optimalBoundingBox().ZMax
        if abs(old_volume - shape.Volume) > 1e-4 or abs(old_bounds.ZMax - new_z) > 1e-5:
            changes[obj.Name] = {"before_volume_mm3": old_volume, "after_volume_mm3": shape.Volume,
                                 "before_max_z_mm": old_bounds.ZMax, "after_max_z_mm": new_z}
    checks["parameter_regression"] = {
        "override": variant_patch, "changed_parts": changes, "exchange": variant_exchange,
        "geometry_pass": variant_checks["pass"],
        "minimum_moving_gap_mm": variant_checks["minimum_moving_gap_mm"],
        "pass": list(changes) == ["Nose"] and variant_checks["pass"]
        and abs(changes["Nose"]["after_max_z_mm"] - changes["Nose"]["before_max_z_mm"] - 12) < 1e-5,
    }
    checks["pass"] = checks["pass"] and checks["parameter_regression"]["pass"]
    dump(destination / "geometry-checks.json", checks)
    dump(variant_dir / "geometry-checks.json", variant_checks)
    App.closeDocument(variant.Name)
    if not checks["pass"]:
        raise RuntimeError("Geometry checks failed; see geometry-checks.json")
    return App.openDocument(str(baseline_path))
