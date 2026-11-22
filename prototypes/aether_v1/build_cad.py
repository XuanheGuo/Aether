"""Create the Aether-V1 FreeCAD model using only native document features."""

from math import cos, pi, sin
from pathlib import Path

import FreeCAD as App
import Part
import Sketcher

V = App.Vector
ROT_X = App.Rotation(V(0, 0, 1), V(1, 0, 0))
ROT_Y = App.Rotation(V(0, 0, 1), V(0, 1, 0))
CERAMIC = (0.82, 0.86, 0.90)
DARK = (0.18, 0.23, 0.29)
TEAL = (0.12, 0.40, 0.48)
METAL = (0.53, 0.58, 0.62)
COPPER = (0.73, 0.35, 0.17)

PARAMETERS = (
    ("Diameter", 240, "Main shell outside diameter"),
    ("HullLength", 1090, "Straight shell axial length"),
    ("Skin", 5, "Straight shell wall thickness"),
    ("NoseLength", 350, "Nose fairing axial length"),
    ("SkirtLength", 145, "Fixed tapered tail housing length"),
    ("HullBase", 265, "Height of straight shell lower edge"),
    ("TailPivot", 109, "Height of both tail pivots"),
    ("FinSpan", 132, "Exposed vane span from its hinge frame"),
    ("FootReach", 212, "Support foot center radius"),
    ("Seam", 2, "Visible seam at hull interfaces"),
    ("FinTravel", "24 deg", "Vane rotation either side of neutral"),
    ("GimbalXTravel", "12 deg", "Outer frame rotation either side of neutral"),
    ("GimbalYTravel", "15 deg", "Inner frame rotation either side of neutral"),
)
R = "(Parameters.Diameter / 2)"
BASE = "Parameters.HullBase"
TOP = "(Parameters.HullBase + Parameters.HullLength)"
NOSE = "Parameters.NoseLength"
PIVOT = "Parameters.TailPivot"


def color_feature(obj, color, density, collision=""):
    obj.addProperty("App::PropertyString", "SimulationRGB", "Simulation")
    obj.SimulationRGB = " ".join(map(str, color))
    obj.addProperty("App::PropertyFloat", "Density", "Simulation", "Concept density, kg/m³")
    obj.Density = density
    obj.addProperty("App::PropertyString", "ContactProxy", "Simulation")
    obj.ContactProxy = collision
    if App.GuiUp:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.LineColor = (0.10, 0.13, 0.17)
        obj.ViewObject.DisplayMode = "Flat Lines"
        obj.ViewObject.Deviation = 0.12
    return obj


def container(doc, name, label, parent=None, position=(0, 0, 0), rotation=None,
              axis=None, limit=None):
    obj = doc.addObject("App::Part", name)
    obj.Label = label
    if parent:
        parent.addObject(obj)
    obj.Placement = App.Placement(V(*(v if isinstance(v, (int, float)) else 0
                                    for v in position)), rotation or App.Rotation())
    for component, value in zip("xyz", position):
        if isinstance(value, str):
            obj.setExpression(f"Placement.Base.{component}", value)
    if axis:
        obj.addProperty("App::PropertyVector", "JointAxis", "Kinematics")
        obj.JointAxis = V(*axis)
        obj.addProperty("App::PropertyAngle", "Travel", "Kinematics")
        obj.setExpression("Travel", "Parameters." + limit)
        obj.addProperty("App::PropertyAngle", "Angle", "Kinematics",
                        "Set to zero before exporting the neutral model")
        obj.Placement = App.Placement(obj.Placement.Base, App.Rotation(V(*axis), 0))
        # Assign a nonzero rotation first so FreeCAD retains the desired axis.
        obj.Placement.Rotation = App.Rotation(V(*axis), 1)
        obj.setExpression("Placement.Rotation.Angle", "Angle")
    return obj


def section(parent, name, position, radii, rotation=None, minor=None):
    """Circle/ring or an ellipse, with its own editable sketch plane."""
    obj = parent.Document.addObject("Sketcher::SketchObject", name)
    parent.addObject(obj)
    obj.Placement = App.Placement(V(*(v if isinstance(v, (int, float)) else 0
                                    for v in position)), rotation or App.Rotation())
    for component, value in zip("xyz", position):
        if isinstance(value, str):
            obj.setExpression(f"Placement.Base.{component}", value)
    if minor is not None:
        index = obj.addGeometry(Part.Ellipse(V(), radii[0], minor), False)
        # Section shape remains editable in Sketcher by releasing this constraint.
        obj.addConstraint(Sketcher.Constraint("Block", index))
    else:
        for radius in radii:
            initial = radius if isinstance(radius, (int, float)) else 50
            index = obj.addGeometry(Part.Circle(V(), V(0, 0, 1), initial), False)
            obj.addConstraint(Sketcher.Constraint("Coincident", index, 3, -1, 1))
            constraint = obj.addConstraint(Sketcher.Constraint("Radius", index, initial))
            if isinstance(radius, str):
                obj.setExpression(f"Constraints[{constraint}]", radius)
    obj.Visibility = False
    return obj


def extrusion(parent, name, label, profile, length, color, density, collision=""):
    obj = parent.Document.addObject("Part::Extrusion", name)
    parent.addObject(obj)
    obj.Label = label
    obj.Base = profile
    obj.DirMode = "Normal"
    obj.LengthFwd = length if isinstance(length, (int, float)) else 10
    if isinstance(length, str):
        obj.setExpression("LengthFwd", length)
    obj.Solid = True
    profile.Visibility = False
    return color_feature(obj, color, density, collision)


def loft(parent, name, label, sections, color, density, collision="", ruled=False):
    obj = parent.Document.addObject("Part::Loft", name)
    parent.addObject(obj)
    obj.Label = label
    obj.Sections = sections
    obj.Solid = True
    obj.Ruled = ruled
    obj.MaxDegree = 3
    for profile in sections:
        profile.Visibility = False
    return color_feature(obj, color, density, collision)


def ring(parent, name, label, position, outer, inner, length, color, density,
         collision="", rotation=None):
    radii = [outer] if inner is None else [outer, inner]
    return extrusion(parent, name, label,
                     section(parent, name + "Section", position, radii, rotation),
                     length, color, density, collision)


def spin_profile(parent, name, label, outline, color, density, collision=""):
    """Spin an editable closed XZ-plane meridian about the local Z axis."""
    doc = parent.Document
    sketch = doc.addObject("Sketcher::SketchObject", name + "Section")
    parent.addObject(sketch)
    sketch.Placement = App.Placement(V(), App.Rotation(V(1, 0, 0), 90))
    points = [V(r, z, 0) for r, z in outline]
    for start, end in zip(points, points[1:] + points[:1]):
        sketch.addGeometry(Part.LineSegment(start, end), False)
    for i in range(len(points)):
        sketch.addConstraint(Sketcher.Constraint("Coincident", i, 2,
                                                 (i + 1) % len(points), 1))
    obj = doc.addObject("Part::Revolution", name)
    parent.addObject(obj)
    obj.Label = label
    obj.Source = sketch
    obj.Axis = V(0, 0, 1)
    obj.Solid = True
    sketch.Visibility = False
    return color_feature(obj, color, density, collision)


def build():
    doc = App.newDocument("AetherV1")
    doc.Label = "Aether V1 · Articulated concept"
    sheet = doc.addObject("Spreadsheet::Sheet", "Parameters")
    sheet.Label = "Concept dimensions"
    sheet.set("A1", "Parameter")
    sheet.set("B1", "Concept value")
    sheet.set("C1", "Description")
    sheet.setBackground("A1:C1", (0.20, 0.42, 0.50))
    sheet.setForeground("A1:C1", (1.0, 1.0, 1.0))
    for row, (name, value, description) in enumerate(PARAMETERS, 2):
        sheet.set(f"A{row}", name)
        sheet.set(f"B{row}", f"{value} mm" if isinstance(value, int) else value)
        sheet.setAlias(f"B{row}", name)
        sheet.set(f"C{row}", description)
    for column, width in (("A", 145), ("B", 130), ("C", 350)):
        sheet.setColumnWidth(column, width)
    doc.recompute()

    root = container(doc, "Vehicle", "Aether V1")
    hull = container(doc, "Airframe", "01  Airframe and fairing", root)
    ring(hull, "MainShell", "Main shell", (0, 0, BASE), R, f"{R} - Parameters.Skin",
         "Parameters.HullLength", CERAMIC, 1550, "cylinder")
    for name, height, width, finish in (
        ("ServiceCollar", f"{BASE} - 20 mm - Parameters.Seam", 20, TEAL),
        ("UpperCollar", f"{TOP} + Parameters.Seam", 13, DARK),
    ):
        ring(hull, name, name.replace("Collar", " collar"), (0, 0, height),
             f"{R} - 0.7 mm", f"{R} - 7 mm", width, finish, 1600)
    nose = container(doc, "Fairing", "Ogive fairing", hull,
                     (0, 0, f"{TOP} + 13 mm + 2 * Parameters.Seam"))
    # A prolate elliptical meridian sampled toward its rounded tip.
    stations = (0, 12, 25, 39, 53, 67, 77, 84, 88, 89.5)
    fairing = loft(nose, "NoseShell", "Smooth nose fairing", [
        section(nose, f"NoseStation{i}", (0, 0, f"{NOSE} * {sin(a*pi/180):.10f}"),
                [f"{R} * {cos(a*pi/180):.10f}"])
        for i, a in enumerate(stations)
    ], CERAMIC, 125, "box")
    fairing.addProperty("App::PropertyString", "MassNote", "Simulation")
    fairing.MassNote = "Effective bulk density for a lightweight fairing envelope"

    vanes = container(doc, "Vanes", "02  Four hinged vanes", root)
    for i, azimuth in enumerate((0, 90, 180, 270), 1):
        s = str(i)
        frame = container(doc, "VaneMount" + s, "Vane " + s + " mount", vanes,
                          (f"{R} * {cos(azimuth*pi/180):.8f}",
                           f"{R} * {sin(azimuth*pi/180):.8f}",
                           f"{BASE} + Parameters.HullLength * 0.78"),
                          App.Rotation(V(0, 0, 1), azimuth))
        loft(frame, "Saddle" + s, "Hinge saddle", [
            section(frame, f"Saddle{s}Station{j}", (x, 0, 0), [a], ROT_X, b)
            for j, (x, a, b) in enumerate(((-4, 93, 25), (7, 80, 22), (16, 24, 17),
                                           (25, 12, 12)))
        ], DARK, 900, ruled=True)
        ring(frame, "Bearing" + s, "Radial bearing", (25, 0, 0), 12, 7.2, 10,
             METAL, 2700, rotation=ROT_X)
        hinge = container(doc, "Vane" + s, "Vane " + s + " hinge", frame,
                          (35, 0, 0), axis=(1, 0, 0), limit="FinTravel")
        ring(hinge, "VanePin" + s, "Hinge pin", (-10, 0, 0), 7, None, 13,
             COPPER, 2700, rotation=ROT_X)
        loft(hinge, "VaneSkin" + s, "Swept vane", [
            section(hinge, f"Vane{s}Station{j}", (f"Parameters.FinSpan * {x}", 0, z),
                    [a], ROT_X, b)
            for j, (x, z, a, b) in enumerate(((0.014, 0, 58, 5), (0.17, -4, 66, 7),
                                               (0.67, -30, 51, 5), (0.92, -45, 29, 3.5),
                                               (1, -50, 8, 1.8)))
        ], TEAL, 540, "box")

    tail = container(doc, "Tail", "03  Two-axis tail assembly", root)
    shell = container(doc, "TailHousing", "Fixed tail housing", tail)
    skirt_levels = (
        (f"{BASE} - 22 mm - Parameters.Seam - Parameters.SkirtLength * 0.56", f"{R} * 0.84"),
        (f"{BASE} - 22 mm - Parameters.Seam - Parameters.SkirtLength * 0.40", f"{R} * 0.90"),
        (f"{BASE} - 22 mm - Parameters.Seam - Parameters.SkirtLength * 0.20", f"{R} * 0.976"),
        (f"{BASE} - 22 mm - Parameters.Seam", R),
    )
    outer = loft(shell, "TailOuter", "Housing outer envelope", [
        section(shell, f"SkirtStation{i}", (0, 0, z), [rad])
        for i, (z, rad) in enumerate(skirt_levels)
    ], DARK, 0)
    bore = ring(shell, "TailBore", "Housing bore tool",
                (0, 0, f"{skirt_levels[0][0]} - 2 mm"), f"{R} * 0.77", None,
                "Parameters.SkirtLength + 5 mm", DARK, 0)
    cut = doc.addObject("Part::Cut", "TailShroud")
    shell.addObject(cut)
    cut.Label = "Open tapered shroud"
    cut.Base, cut.Tool = outer, bore
    outer.Visibility = bore.Visibility = False
    color_feature(cut, DARK, 1200)
    for side in (-1, 1):
        s = "L" if side < 0 else "R"
        bearing_rot = App.Rotation(V(0, 0, 1), V(side, 0, 0))
        loft(shell, "TailFork" + s, "Fixed bearing arm " + s, [
            section(shell, f"Fork{s}Station{i}", (f"{R} * {side * x}", 0, z), [a],
                    minor=b)
            for i, (x, z, a, b) in enumerate(((0.79, f"{PIVOT} - 17 mm", 9, 7),
                                              (0.82, PIVOT, 11, 9),
                                              (0.86, f"{BASE} - 22 mm - Parameters.Seam - Parameters.SkirtLength * 0.52", 13, 10)))
        ], DARK, 1500)
        ring(shell, "OuterBearing" + s, "Outer trunnion bearing " + s,
             (side * 84, 0, PIVOT), 12, 7.2, 16, METAL, 2700, rotation=bearing_rot)

    gimbal = container(doc, "GimbalX", "Outer gimbal · X", tail, (0, 0, PIVOT),
                       axis=(1, 0, 0), limit="GimbalXTravel")
    spin_profile(gimbal, "GimbalRing", "Outer gimbal ring",
                 [(72, -8), (77, -11), (80, -7), (80, 7),
                  (77, 11), (72, 8), (69, 5), (69, -5)], TEAL, 1850)
    nozzle = container(doc, "GimbalY", "Inner nozzle cradle · Y", gimbal,
                       axis=(0, 1, 0), limit="GimbalYTravel")
    spin_profile(nozzle, "NozzleBell", "Inert display nozzle",
                 [(35, 12), (38, 6), (38, -14), (41, -36), (48, -58),
                  (57, -76), (57, -79), (52, -84), (46, -84), (44, -74),
                  (37, -56), (31, -35), (28, -14), (28, 10)], METAL, 2200)
    for side in (-1, 1):
        s = "L" if side < 0 else "R"
        for parent, name, x, y, axis, radius, inner, length, color in (
            (gimbal, "OuterPin", 79, 0, V(side, 0, 0), 7, None, 20, COPPER),
            (gimbal, "InnerBearing", 0, 64, V(0, side, 0), 10, 6.1, 9, METAL),
            (nozzle, "InnerPin", 0, 37, V(0, side, 0), 6, None, 35, COPPER),
        ):
            ring(parent, name + s, name + " " + s, (side*x, side*y, 0), radius,
                 inner, length, color, 2700,
                 rotation=App.Rotation(V(0, 0, 1), axis))

    supports = container(doc, "Supports", "04  Static display supports", root)
    for i, azimuth in enumerate((45, 135, 225, 315), 1):
        s = str(i)
        leg = container(doc, "Stand" + s, "Support " + s, supports,
                        rotation=App.Rotation(V(0, 0, 1), azimuth))
        loft(leg, "Strut" + s, "Swept support strut", [
            section(leg, f"Strut{s}Station{j}", (x, 0, z), [a], minor=b)
            for j, (x, z, a, b) in enumerate((
                ("Parameters.FootReach", 12, 15, 13),
                ("Parameters.FootReach - 21 mm", 65, 19, 12),
                (f"{R} + 28 mm", f"{BASE} * 0.64", 20, 12),
                (f"{R} - 1 mm", f"{BASE} + 31 mm", 14, 10),
                (f"{R} - 1 mm", f"{BASE} + 79 mm", 8, 6),
            ))
        ], DARK, 830)
        loft(leg, "Foot" + s, "Floor contact pad", [
            section(leg, f"Foot{s}Station{j}", ("Parameters.FootReach", 0, z),
                    [a], minor=b)
            for j, (z, a, b) in enumerate(((0, 23, 17), (4, 26, 19), (12, 18, 15)))
        ], TEAL, 1000, "box", ruled=True)

    doc.recompute()
    # Linked features may hide or show their inputs during creation.
    for obj in doc.Objects:
        if obj.TypeId == "Sketcher::SketchObject" or (
            "Density" in obj.PropertiesList and obj.Density == 0
        ):
            obj.Visibility = False
    return doc


def set_view(view, azimuth=27, inclination=80):
    """Set the orthographic camera directly before saving or capturing it."""
    from pivy import coin

    view.setCameraType("Orthographic")
    orientation = App.Rotation(V(0, 0, 1), azimuth).multiply(
        App.Rotation(V(1, 0, 0), inclination))
    view.getCameraNode().orientation.setValue(coin.SbRotation(*orientation.Q))
    view.fitAll()


def main(destination):
    from export_sim import export

    destination = Path(destination)
    doc = build()
    export(doc, destination)
    if App.GuiUp:
        import FreeCADGui as Gui

        set_view(Gui.activeDocument().activeView())
    doc.saveAs(str(destination / "Aether-V1.FCStd"))
    return doc
