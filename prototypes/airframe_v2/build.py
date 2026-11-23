"""Native FreeCAD primitives, booleans, lofts and expression-driven assemblies."""
import json
from pathlib import Path

import FreeCAD as App

V = App.Vector
ROOT = Path(__file__).resolve().parent
COLORS = {
    "shell": (0.65, 0.73, 0.78),
    "nose": (0.56, 0.65, 0.69),
    "collar": (0.24, 0.32, 0.37),
    "panel": (0.22, 0.49, 0.54),
    "ring": (0.76, 0.46, 0.22),
    "insert": (0.30, 0.52, 0.57),
    "support": (0.31, 0.38, 0.43),
    "envelope": (0.47, 0.64, 0.69),
}
DESCRIPTIONS = (
    "Main shell outside radius", "Straight shell length", "Nominal shell wall",
    "Nose semiaxis along Z", "Tail fairing generating ellipsoid axial radius",
    "Tail fairing cylindrical opening radius", "Panel pivot station from rear seam",
    "Panel pivot offset from shell OD", "Panel span measured from pivot",
    "Full root section chord", "Full root section thickness", "Tail pivot station",
    "Outer tail frame radius", "Outer tail frame radial width",
    "Outer tail frame axial depth", "Inert inner frame radius",
)


def make(parent, kind, name, label=None, **values):
    doc = parent.Document if hasattr(parent, "Document") else parent
    obj = doc.addObject(kind, name)
    if hasattr(parent, "Document"):
        parent.addObject(obj)
    obj.Label = label or name
    for prop, value in values.items():
        if isinstance(value, str):
            obj.setExpression(prop.replace("_", "."), value)
        else:
            setattr(obj, prop, value)
    return obj


def finished(obj, style, role="physical"):
    obj.addProperty("App::PropertyString", "ExportRole", "Round 1")
    obj.ExportRole = role
    if App.GuiUp:
        obj.ViewObject.ShapeColor = COLORS[style]
        obj.ViewObject.LineColor = (0.13, 0.19, 0.23)
        obj.ViewObject.DisplayMode = "Flat Lines"
        obj.ViewObject.Deviation = 0.07
        obj.ViewObject.AngularDeflection = 8
    return obj


def cut(parent, name, label, additive, subtractive, style):
    return finished(make(parent, "Part::Cut", name, label, Base=additive,
                         Tool=subtractive), style)


def frame(parent, name, label, axis=None, **values):
    obj = make(parent, "App::Part", name, label, **values)
    if axis:
        obj.addProperty("App::PropertyVector", "JointAxis", "Mechanical display")
        obj.JointAxis = V(*axis)
        obj.addProperty("App::PropertyAngle", "Angle", "Mechanical display")
        obj.addProperty("App::PropertyAngle", "DisplayLimit", "Mechanical display")
        obj.DisplayLimit = 12
        obj.Placement.Rotation = App.Rotation(V(*axis), 1)
        obj.setExpression("Placement.Rotation.Angle", "Angle")
    return obj


def build(overrides=None):
    values = json.loads((ROOT / "parameters/baseline.json").read_text())
    if overrides:
        unknown = overrides.keys() - values.keys()
        if unknown:
            raise ValueError(f"Unknown parameters: {sorted(unknown)}")
        values.update(overrides)
    doc = App.newDocument("AirframeV2")
    doc.Label = "Airframe-V2 · Round 1"
    table = doc.addObject("Spreadsheet::Sheet", "Parameters")
    table.Label = "Round 1 · Concept dimensions"
    for col, heading, width in (("A", "Parameter", 165), ("B", "Concept value", 115),
                                ("C", "Description · assumed", 355)):
        table.set(f"{col}1", heading)
        table.setColumnWidth(col, width)
    table.setStyle("A1:C1", "bold", "add")
    for row, ((name, value), description) in enumerate(zip(values.items(), DESCRIPTIONS), 2):
        table.set(f"A{row}", name)
        table.set(f"B{row}", value)
        table.setAlias(f"B{row}", name)
        table.set(f"C{row}", description)

    root = frame(doc, "Airframe", "Airframe-V2 assembly")
    body = frame(root, "Body", "01 · Shell and detachable nose")
    outer = make(body, "Part::Cylinder", "ShellOuter",
                 Radius="Parameters.ShellRadius", Height="Parameters.ShellLength")
    bore = make(body, "Part::Cylinder", "ShellBore",
                Radius="Parameters.ShellRadius - Parameters.Wall",
                Height="Parameters.ShellLength + 2 mm", Placement_Base_z="-1 mm")
    cut(body, "Shell", "Hollow main shell", outer, bore, "shell")

    nose = frame(body, "NoseFrame", "Detachable nose local frame",
                 Placement_Base_z="Parameters.ShellLength")
    for name, inset in (("NoseOuter", ""), ("NoseInner", " - Parameters.Wall")):
        make(nose, "Part::Ellipsoid", name,
             Radius1="Parameters.NoseLength" + inset,
             Radius2="Parameters.ShellRadius" + inset,
             Radius3="Parameters.ShellRadius" + inset,
             Angle1=0, Angle2=90)
    cut(nose, "Nose", "Tangent ellipsoidal nose shell", doc.NoseOuter,
        doc.NoseInner, "nose")

    panels = frame(root, "Panels", "02 · Independent exterior panels")
    section_rotation = App.Rotation(V(0, 1, 0), 90)
    for index in range(4):
        mounting = frame(panels, f"Mount{index}", f"Panel {index + 1} · azimuth station",
                         Placement=App.Placement(V(), App.Rotation(V(0, 0, 1), 90 * index)),
                         Placement_Base_z="Parameters.PanelStation")
        pod = make(mounting, "Part::Box", f"Pod{index}", "Fixed support envelope",
                   Length="Parameters.PanelStandOff", Width=18, Height=50,
                   Placement_Base_x="Parameters.ShellRadius - 2 mm",
                   Placement_Base_y="-9 mm", Placement_Base_z="-25 mm")
        finished(pod, "support")
        hinge = frame(mounting, f"PanelJoint{index}", "Radial hinge display frame",
                      axis=(1, 0, 0),
                      Placement_Base_x="Parameters.ShellRadius + Parameters.PanelStandOff")
        sections = []
        for station, radial, shift, chord, thickness in (
            (0, "3 mm", "0 mm", 0.40, 0.40),
            (1, "0.35 * Parameters.PanelSpan", "-5 mm", 0.50, 0.50),
            (2, "0.78 * Parameters.PanelSpan", "-22 mm", 0.37, 0.34),
            (3, "Parameters.PanelSpan", "-34 mm", 0.19, 0.19),
        ):
            sections.append(make(
                hinge, "Part::Ellipse", f"Panel{index}Section{station}",
                Placement=App.Placement(V(), section_rotation),
                Placement_Base_x=radial, Placement_Base_z=shift,
                MajorRadius=f"{chord} * Parameters.PanelRootChord",
                MinorRadius=f"{thickness} * Parameters.PanelThickness", Angle2=360))
        finished(make(hinge, "Part::Loft", f"Panel{index}", "Rounded section panel",
                      Sections=sections, Solid=True), "panel")

    tail = frame(root, "Tail", "03 · Inert two-axis tail demonstrator")
    fairing = make(tail, "Part::Ellipsoid", "CollarOuter", Angle1=-35, Angle2=0,
                   Radius1="Parameters.CollarLength",
                   Radius2="Parameters.ShellRadius", Radius3="Parameters.ShellRadius")
    collar_bore = make(tail, "Part::Cylinder", "CollarBore",
                       Radius="Parameters.CollarBore", Height="2 * Parameters.CollarLength",
                       Placement_Base_z="-Parameters.CollarLength")
    cut(tail, "Collar", "Fixed rear fairing", fairing, collar_bore, "collar")
    outer_joint = frame(tail, "TailX", "Outer frame · X", axis=(1, 0, 0),
                        Placement_Base_z="Parameters.TailStation")
    ring_outer = make(outer_joint, "Part::Cylinder", "RingOuter",
                      Radius="Parameters.RingRadius", Height="Parameters.RingDepth",
                      Placement_Base_z="-Parameters.RingDepth / 2")
    ring_inner = make(outer_joint, "Part::Cylinder", "RingInner",
                      Radius="Parameters.RingRadius - Parameters.RingWidth",
                      Height="Parameters.RingDepth + 2 mm",
                      Placement_Base_z="-Parameters.RingDepth / 2 - 1 mm")
    cut(outer_joint, "Ring", "Outer display ring", ring_outer, ring_inner, "ring")
    inner_joint = frame(outer_joint, "TailY", "Inner frame · local Y", axis=(0, 1, 0))
    finished(make(inner_joint, "Part::Ellipsoid", "Insert", "Inert tail insert",
                  Radius1="0.7 * Parameters.InsertRadius",
                  Radius2="Parameters.InsertRadius", Radius3="0.8 * Parameters.InsertRadius"),
             "insert")

    support = frame(root, "Support", "04 · Internal support and volume envelopes")
    for index, z in enumerate(("0.24 * Parameters.ShellLength", "0.62 * Parameters.ShellLength")):
        for sign in (-1, 1):
            finished(make(support, "Part::Box", f"Rail{index}{'L' if sign < 0 else 'R'}",
                          "Internal support rail", Length=9, Width=12, Height=118,
                          Placement_Base_x=f"{-55 if sign < 0 else 46} mm",
                          Placement_Base_y="-32 mm", Placement_Base_z=z), "support")
    for name, label, fraction, size in (
        ("VolumeA", "Reserved equipment volume A", 0.24, (82, 55, 108)),
        ("VolumeB", "Reserved equipment volume B", 0.62, (82, 64, 108)),
    ):
        length, width, height = size
        finished(make(support, "Part::Box", name, label, Length=length, Width=width,
                      Height=height, Placement_Base_x=f"{-length / 2} mm",
                      Placement_Base_y="-24 mm",
                      Placement_Base_z=f"{fraction} * Parameters.ShellLength + 5 mm"),
                 "envelope", "envelope")
    doc.recompute()
    return doc


def parts(doc):
    return [obj for obj in doc.Objects if "ExportRole" in obj.PropertiesList]


def joints(doc):
    return [obj for obj in doc.Objects if "JointAxis" in obj.PropertiesList]


def world_shape(obj):
    shape = obj.Shape.copy()
    shape.Placement = obj.getGlobalPlacement()
    return shape


def visibility(doc):
    for obj in doc.Objects:
        if obj.TypeId == "App::Part":
            obj.Visibility = True
    for obj in doc.Objects:
        if obj.TypeId != "App::Part" and hasattr(obj, "Shape"):
            obj.Visibility = "ExportRole" in obj.PropertiesList and obj.ExportRole == "physical"
    doc.Support.Visibility = False
