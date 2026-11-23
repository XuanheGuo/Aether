"""Check static STL import and reconstructed world bounds in MuJoCo."""
import argparse
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def numbers(values):
    return " ".join(str(value) for value in values)


def smoke(directory):
    manifest = json.loads((directory / "assembly.json").read_text())
    scale = manifest["mm_to_m"]
    scene = ET.Element("mujoco", model="Airframe V2 geometry smoke")
    assets = ET.SubElement(scene, "asset")
    world = ET.SubElement(scene, "worldbody")
    files = {}
    expected = {}
    for name, entry in manifest["parts"].items():
        pose = entry["world_from_part_neutral"]
        xyz = np.array(pose["translation_mm"]) * scale
        quaternion = np.array(pose["quaternion_wxyz"])
        filename = name + ".stl"
        stl = (directory / entry["visual_stl"]).read_bytes()
        files[filename] = stl
        ET.SubElement(assets, "mesh", name=name, file=filename, scale=numbers([scale] * 3))
        ET.SubElement(world, "geom", name=name, type="mesh", mesh=name, pos=numbers(xyz),
                      quat=numbers(quaternion), contype="0", conaffinity="0")
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, quaternion)
        vertices = np.asarray([record[3:12] for record in struct.iter_unpack("<12fH", stl[84:])])
        expected[name] = (vertices.reshape(-1, 3) * scale) @ matrix.reshape(3, 3).T + xyz

    model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"), files)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    errors = {}
    for name, vertices in expected.items():
        geom = model.geom(name)
        mesh_id = geom.dataid[0]
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        mesh = model.mesh_vert[start:start + count]
        world_vertices = mesh @ data.geom_xmat[geom.id].reshape(3, 3).T + data.geom_xpos[geom.id]
        errors[name] = max(float(np.max(np.abs(reduce(world_vertices, axis=0) - reduce(vertices, axis=0))))
                           for reduce in (np.min, np.max))
    error = max(errors.values())
    if error > 1e-5:
        raise RuntimeError(f"World bounds differ by {error} m")
    return {"mujoco_version": mujoco.__version__, "visual_meshes": model.nmesh,
            "world_geom_bounds_max_delta_m": error, "bounds_delta_m_by_part": errors,
            "pass": True}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    print(json.dumps(smoke(parser.parse_args().directory), indent=2, sort_keys=True))
