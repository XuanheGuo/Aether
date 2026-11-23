"""Discrete BRep clearance, seam and parameter persistence checks."""
from itertools import combinations, product
from math import hypot

import FreeCAD as App
import Part

from build import joints, parts, world_shape

V = App.Vector
VOLUME_TOL = 1e-4
GAP_TOL = 1e-5


def valid_shapes(doc):
    doc.recompute()
    invalid = [obj.Name for obj in doc.Objects if "Invalid" in obj.State]
    if invalid:
        raise ValueError(f"Invalid feature states: {invalid}")
    shapes = {obj.Name: world_shape(obj) for obj in parts(doc)}
    for name, shape in shapes.items():
        if shape.isNull() or not shape.isValid() or len(shape.Solids) != 1 or shape.Volume <= 0:
            raise ValueError(f"Invalid single solid: {name}")
    return shapes


def verify(doc):
    frames = joints(doc)
    moving = {f"Panel{i}" for i in range(4)} | {"Ring", "Insert"}
    expected_mount = {frozenset(("Shell", f"Pod{i}")) for i in range(4)}
    moving_min = {}
    mount_overlap = {}
    static_min = {}
    pose_records = []
    conservative_distance_fallbacks = []
    cached_pairs = {}
    dependencies = {f"Panel{i}": (i,) for i in range(4)}
    dependencies.update(Ring=(4,), Insert=(4, 5))
    failures = []
    axis_samples = (-12, -6, 0, 6, 12)
    cases = [(0,) * 6]
    cases += [(0, 0, 0, 0, x, y) for x, y in product(axis_samples, repeat=2) if x or y]
    cases += [tuple(angle if i == n else 0 for i in range(6))
              for n in range(4) for angle in axis_samples if angle]
    cases += [(12, -12, 12, -12, 12, -12), (-12, 12, -12, 12, -12, 12)]
    try:
        for pose in cases:
            for frame, angle in zip(frames, pose):
                frame.Angle = angle
            shapes = valid_shapes(doc)
            if pose == cases[0]:
                neutral = shapes
            bounds = {name: shape.BoundBox for name, shape in shapes.items()}
            min_gap = float("inf")
            for a, b in combinations(shapes, 2):
                is_moving = a in moving or b in moving
                if pose != cases[0] and not is_moving:
                    continue
                key = f"{a}/{b}"
                angles = tuple(pose[i] for i in dependencies.get(a, ()) + dependencies.get(b, ()))
                cache_key = (a, b, angles)
                if cache_key not in cached_pairs:
                    box_a, box_b = bounds[a], bounds[b]
                    lower = hypot(*(max(getattr(box_a, axis + "Min") - getattr(box_b, axis + "Max"),
                                        getattr(box_b, axis + "Min") - getattr(box_a, axis + "Max"), 0)
                                    for axis in "XYZ"))
                    overlap = shapes[a].common(shapes[b]).Volume if box_a.intersect(box_b) else 0
                    gap = lower if lower >= 10 else shapes[a].distToShape(shapes[b])[0]
                    if gap + GAP_TOL < lower:
                        conservative_distance_fallbacks.append(
                            {"pair": key, "angles_deg": pose, "raw_mm": gap, "lower_bound_mm": lower})
                        gap = lower
                    cached_pairs[cache_key] = gap, overlap
                gap, overlap = cached_pairs[cache_key]
                if frozenset((a, b)) in expected_mount:
                    mount_overlap[key] = overlap
                    if overlap <= VOLUME_TOL:
                        failures.append([key, "missing intended mounting envelope overlap", overlap])
                    continue
                if overlap > VOLUME_TOL:
                    failures.append([key, list(pose), overlap])
                if is_moving:
                    min_gap = min(min_gap, gap)
                    moving_min[key] = min(moving_min.get(key, gap), gap)
                    if gap <= GAP_TOL:
                        failures.append([key, list(pose), "moving surface touches"])
                else:
                    static_min[key] = gap
            pose_records.append({"angles_deg": pose, "minimum_moving_gap_mm": min_gap})
    finally:
        for frame in frames:
            frame.Angle = 0
        doc.recompute()
    radius = doc.Parameters.get("B2").Value
    z = doc.Parameters.get("B3").Value
    epsilon = 0.01
    seams = {}
    # Intersect radial rays immediately either side of each planar seam. The
    # exterior radius and two one-sided slopes expose a step or tangent break.
    for name, station, aft, fore in (("nose", z, "Shell", "Nose"),
                                     ("tail", 0, "Collar", "Shell")):
        radii = {}
        for part, offset in ((aft, -epsilon), (aft, -2 * epsilon),
                             (fore, epsilon), (fore, 2 * epsilon)):
            ray = Part.makeLine(V(0, 0, station + offset), V(2 * radius, 0, station + offset))
            radii[offset] = max(vertex.Point.x for vertex in neutral[part].common(ray).Vertexes)
        slope_a = (radii[-epsilon] - radii[-2 * epsilon]) / epsilon
        slope_b = (radii[2 * epsilon] - radii[epsilon]) / epsilon
        gap = static_min[f"{aft}/{fore}"] if f"{aft}/{fore}" in static_min else static_min[f"{fore}/{aft}"]
        seams[name] = {"gap_mm": gap, "near_seam_radius_step_mm": abs(radii[-epsilon] - radii[epsilon]),
                       "near_seam_slope_delta": abs(slope_a - slope_b)}
        if gap > GAP_TOL or seams[name]["near_seam_radius_step_mm"] > 0.001 or abs(slope_a - slope_b) > 0.001:
            failures.append([name, "seam continuity outside sampled tolerance"])
    return {
        "freecad_version": ".".join(App.Version()[:3]),
        "valid_single_solids": len(neutral),
        "physical_solids": sum(obj.ExportRole == "physical" for obj in parts(doc)),
        "joint_order": [frame.Name for frame in frames],
        "poses": pose_records,
        "minimum_moving_gap_mm": min(moving_min.values()),
        "minimum_moving_gap_by_pair_mm": moving_min,
        "static_neutral_gap_by_pair_mm": static_min,
        "declared_static_mount_overlap_mm3": mount_overlap,
        "gap_method": "BRep surface distance near interfaces; conservative box spacing at or above 10 mm",
        "conservative_distance_fallbacks": conservative_distance_fallbacks,
        "seams": seams, "unexpected_interference_or_contact": failures,
        "pass": not failures,
    }
