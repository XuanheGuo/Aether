"""Capture native FreeCAD viewports, including two temporary geometry views."""
from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui
import Part
from pivy import coin

from build import joints, parts, visibility, world_shape

V = App.Vector


def capture(doc, path, direction, right=(0, 0, 1), square=False):
    App.setActiveDocument(doc.Name)
    view = Gui.getDocument(doc.Name).activeView()
    view.setCameraType("Orthographic")
    normal = V(*direction)
    x = V(*right)
    rotation = App.Rotation(x, normal.cross(x), normal, "ZXY")
    camera = view.getCameraNode()
    camera.orientation.setValue(coin.SbRotation(*rotation.Q))
    view.fitAll()
    camera.height.setValue(camera.height.getValue() * (0.70 if square else 0.76))
    Gui.updateGui()
    width, height = (1000, 1000) if square else (1500, 850)
    view.saveImage(str(path), width, height, "White")
    if not path.is_file() or path.stat().st_size < 1024:
        raise RuntimeError(f"Viewport image missing: {path}")


def derived_view(doc, path, section=False):
    source = [obj for obj in parts(doc)
              if section or obj.getParentGeoFeatureGroup() != doc.Support]
    temporary = App.newDocument("Cutaway" if section else "AssemblyLayout")
    slice_tool = Part.makeBox(1000, 500, 1500, V(-500, 0, -250)) if section else None
    try:
        offsets = {"Nose": V(0, 0, 110), "Collar": V(0, 0, -55),
                   "Ring": V(0, 0, -119), "Insert": V(0, 0, -179)}
        for obj in source:
            shape = world_shape(obj)
            if section and obj.ExportRole == "physical":
                shape = shape.cut(slice_tool)
            if shape.isNull() or not shape.Solids:
                continue
            if not section:
                if obj.Name.startswith("Panel"):
                    origin = obj.getGlobalPlacement().Base
                    vector = V(origin.x, origin.y, 0)
                    vector.normalize()
                    shape.translate(vector * 39)
                elif obj.Name in offsets:
                    shape.translate(offsets[obj.Name])
            shown = temporary.addObject("Part::Feature", obj.Name)
            shown.Label = obj.Label
            shown.Shape = shape
            shown.ViewObject.ShapeColor = obj.ViewObject.ShapeColor
            shown.ViewObject.LineColor = (0.12, 0.18, 0.22)
            shown.ViewObject.DisplayMode = "Flat Lines"
            shown.ViewObject.Deviation = 0.07
        temporary.recompute()
        if section:
            capture(temporary, path, (0, 1, 0))
        else:
            capture(temporary, path, (1.5, -2.7, 1.15))
    finally:
        App.closeDocument(temporary.Name)
        App.setActiveDocument(doc.Name)


def render(doc, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    visibility(doc)
    capture(doc, directory / "front.png", (0, 0, 1), (1, 0, 0), square=True)
    capture(doc, directory / "side.png", (0, -1, 0))
    capture(doc, directory / "top.png", (1, 0, 0))
    derived_view(doc, directory / "section.png", section=True)
    derived_view(doc, directory / "assembly.png")
    try:
        for prefix, angles in (
            ("pose-a", (12, -12, 12, -12, 12, -12)),
            ("pose-b", (-12, 12, -12, 12, -12, 12)),
        ):
            for frame, angle in zip(joints(doc), angles):
                frame.Angle = angle
            doc.recompute()
            capture(doc, directory / f"{prefix}.png", (1.55, -2.8, -1.45))
    finally:
        for frame in joints(doc):
            frame.Angle = 0
        doc.recompute()
    capture(doc, directory / "axonometric.png", (1.55, -2.8, -1.45))
