"""Round-trip regression for frozen CAD, regenerated meshes, and grounded MuJoCo."""

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

PROTOTYPE = Path(__file__).resolve().parents[1]
FREECAD_PYTHON = Path("/Applications/FreeCAD.app/Contents/Resources/bin/python")
HINGES = ("Vane1", "Vane2", "Vane3", "Vane4", "GimbalX", "GimbalY")
FEET = {f"Foot{i}_contact" for i in range(1, 5)}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def hashes(folder):
    return {str(path.relative_to(folder)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(folder.rglob("*")) if path.is_file()}


def check_structure(model, cad, folder):
    manifest = json.loads((folder / "assembly.json").read_text())
    require(set(manifest["parts"]) == set(cad["parts"]), "CAD/export part names differ.")
    require(set(manifest["joints"]) == set(cad["joints"]) == set(HINGES),
            "CAD/export hinge names differ.")
    visual = {model.geom(i).name for i in range(model.ngeom)
              if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH}
    require(visual == set(cad["parts"]) and model.nmesh == len(visual) == 41,
            "Expected all 41 CAD visual meshes.")
    require(model.njnt == 7 and model.nu == len(HINGES), "Unexpected joint or actuator count.")
    require(int(model.joint("Base").type[0]) == mujoco.mjtJoint.mjJNT_FREE,
            "Vehicle base is not free.")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    max_bounding_error = 0.0
    for part, expected in cad["parts"].items():
        metadata = manifest["parts"][part]
        require(metadata["body"] == expected["parent"]
                and math.isclose(metadata["volume_mm3"], expected["volume_mm3"],
                                 rel_tol=0, abs_tol=0.001),
                f"Export manifest does not match CAD for {part}.")
        geom = model.geom(part)
        mesh = model.mesh(part)
        require(geom.bodyid[0] == model.body(expected["parent"]).id
                and geom.dataid[0] == mesh.id
                and geom.contype[0] == 0 and geom.conaffinity[0] == 0,
                f"Visual mesh {part} is attached incorrectly or contributes to collisions.")
        first, count = model.mesh_vertadr[mesh.id], model.mesh_vertnum[mesh.id]
        # mjData geometry transforms include the compiler's offset and rotation of mesh vertices.
        rotation = data.geom_xmat[geom.id].reshape(3, 3)
        vertices = model.mesh_vert[first:first + count] @ rotation.T + data.geom_xpos[geom.id]
        bounds = np.array([vertices.min(axis=0), vertices.max(axis=0)]) * 1000
        # Surface bounds differ slightly after discretizing the B-Rep curves.
        bounding_error = float(np.max(abs(bounds - expected["bounds_mm"])))
        require(bounding_error <= 0.6, f"{part}: CAD-to-mesh bounds differ by {bounding_error:g} mm.")
        max_bounding_error = max(max_bounding_error, bounding_error)
    require(set(FEET) <= {model.geom(i).name for i in range(model.ngeom)},
            "Ground contact foot geometry is missing.")
    for name, expected in cad["joints"].items():
        joint = model.joint(name)
        body = model.body(name)
        limit = expected["travel_deg"]
        require(int(joint.type[0]) == mujoco.mjtJoint.mjJNT_HINGE
                and int(joint.bodyid[0]) == body.id
                and int(body.parentid[0]) == model.body(expected["parent"]).id
                and bool(joint.limited[0])
                and np.allclose(joint.axis, expected["axis"], rtol=0, atol=1e-10)
                and np.allclose(np.rad2deg(joint.range), [-limit, limit], rtol=0, atol=1e-9),
                f"Joint mismatch: {name}")

    position_error = rotation_error = 0.0
    for pose in cad["poses"]:
        for name, angle in pose["commands_deg"].items():
            data.qpos[model.joint(name).qposadr[0]] = math.radians(angle)
        mujoco.mj_forward(model, data)
        for name, expected in pose["world"].items():
            body = data.body(name)
            position_error = max(position_error, float(np.linalg.norm(
                body.xpos - np.array(expected["position_mm"]) / 1000)))
            actual = Rotation.from_quat(body.xquat, scalar_first=True)
            reference = Rotation.from_quat(expected["quat_wxyz"], scalar_first=True)
            rotation_error = max(rotation_error, float((actual * reference.inv()).magnitude()))
    require(position_error <= 1e-9 and rotation_error <= 1e-9, "CAD/MJCF world pose mismatch.")
    mass = model.body_mass[1:]
    inertia = np.sort(model.body_inertia[1:], axis=1)
    require(np.all(np.isfinite(mass)) and np.all(mass > 0)
            and np.all(np.isfinite(inertia)) and np.all(inertia > 0)
            and np.all(inertia[:, 2] < inertia[:, 0] + inertia[:, 1]),
            "Dynamic body mass or principal inertia is invalid.")
    return {"visual_meshes": len(visual), "hinges": len(cad["joints"]),
            "pose_samples": len(cad["poses"]), "max_world_bounds_error_mm": max_bounding_error,
            "max_position_error_m": position_error, "max_rotation_error_rad": rotation_error,
            "total_concept_mass_kg": float(sum(mass))}


def check_state(data):
    require(not np.any(data.warning.number) and np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel)), "MuJoCo state is nonfinite or has warnings.")


def check_contacts(model):
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    steps = round(5 / dt)
    required_samples = round(1 / dt)
    base_positions = []
    feet_count = {name: 0 for name in FEET}
    max_angle = 0.0
    for step in range(steps):
        mujoco.mj_step(model, data)
        check_state(data)
        if step >= steps - required_samples:
            touching_floor = set()
            for contact in data.contact:
                names = {model.geom(geom).name for geom in contact.geom}
                if "floor" in names:
                    touching_floor |= names & FEET
            for name in touching_floor:
                feet_count[name] += 1
            base_positions.append(data.qpos[:3].copy())
            max_angle = max(max_angle, float(Rotation.from_quat(
                data.qpos[3:7], scalar_first=True).magnitude()))
    stable_feet = {name: count / required_samples for name, count in sorted(feet_count.items())}
    positions = np.array(base_positions)
    drift = float(np.max(np.linalg.norm(positions - positions[0], axis=1)))
    max_height_change = float(np.max(abs(positions[:, 2])))
    require(min(stable_feet.values()) >= 0.95 and drift < 0.002
            and max_height_change < 0.005 and max_angle < math.radians(1),
            f"Ground contact stability failed: {stable_feet}, drift {drift:g} m.")
    return {"duration_s": round(float(data.time), 6), "evaluated_final_s": 1,
            "foot_contact_fraction": stable_feet, "max_base_position_drift_m": drift,
            "max_base_height_change_m": max_height_change, "max_base_rotation_deg":
            math.degrees(max_angle)}


def check_actuators(model):
    responses = {}
    for name in HINGES:
        data = mujoco.MjData(model)
        joint = model.joint(name)
        actuator = model.actuator(name + "_target").id
        require(int(model.actuator_trnid[actuator, 0]) == joint.id,
                f"Actuator {name} is wired to another joint.")
        address = joint.qposadr[0]
        limit = float(joint.range[1])
        dt = model.opt.timestep
        for _ in range(round(1 / dt)):
            mujoco.mj_step(model, data)
            check_state(data)
        responses[name] = []
        for sign in (1, -1):
            start = float(data.qpos[address])
            target = sign * 0.7 * limit
            data.ctrl[actuator] = target
            for _ in range(round(3 / dt)):
                mujoco.mj_step(model, data)
                check_state(data)
            final = float(data.qpos[address])
            error = abs(math.degrees(final - target))
            require(sign * final > 0 and sign * (final - start) > 0
                    and error <= 6 and abs(final) <= limit + math.radians(0.2),
                    f"{name} target {math.degrees(target):g}° missed: {math.degrees(final):g}°.")
            responses[name].append({"start_deg": math.degrees(start),
                                    "target_deg": math.degrees(target),
                                    "final_deg": math.degrees(final), "error_deg": error})
    return responses


def check_variant(folder, cad):
    model = mujoco.MjModel.from_xml_path(str(folder / "aether_v1.xml"))
    return {"structure": check_structure(model, cad, folder), "ground": check_contacts(model),
            "actuator_error_limit_deg": 6, "actuators": check_actuators(model)}


def regression(freecad_python, source):
    before = hashes(source)
    try:
        with tempfile.TemporaryDirectory(prefix="aether-v1-") as directory:
            scratch = Path(directory)
            baseline = scratch / "baseline"
            edited = scratch / "edited"
            shutil.copytree(source, baseline)
            subprocess.run(
                [str(freecad_python), "-B", str(Path(__file__).with_name("cad_probe.py")),
                 str(baseline / "Aether-V1.FCStd"), str(edited),
                 "--library", str(freecad_python.parent.parent / "lib")],
                check=True,
            )
            cad = json.loads((edited / "cad_probe.json").read_text())
            reports = {variant: check_variant(folder, cad[variant])
                       for variant, folder in (("baseline", baseline), ("edited", edited))}
            changed_volumes = {
                name for name, part in cad["baseline"]["parts"].items()
                if not math.isclose(part["volume_mm3"], cad["edited"]["parts"][name]["volume_mm3"],
                                    rel_tol=1e-8, abs_tol=1e-5)
            }
            expected = {f"VaneSkin{i}" for i in range(1, 5)}
            require(changed_volumes == expected, f"Unexpected changed solids: {changed_volumes}")
            edited_hashes = hashes(edited)
            require(all(before[f"meshes/{name}.stl"] != edited_hashes[f"meshes/{name}.stl"]
                        for name in expected), "Expected four newly tessellated vane meshes.")
            require(before["aether_v1.xml"] != edited_hashes["aether_v1.xml"],
                    "MJCF should change with the updated inertia and contact geometry.")
    finally:
        require(hashes(source) == before, "Frozen CAD/MJCF bundle changed during the regression.")
    return {
        "freecad": cad["freecad"], "mujoco": mujoco.__version__,
        "parameter_edit": cad["edit"], "changed_solids": sorted(changed_volumes),
        "original_bundle_unchanged": True, "original_file_sha256": before,
        "edited_cad_sha256": edited_hashes["Aether-V1.FCStd"],
        "edited_mjcf_sha256": edited_hashes["aether_v1.xml"],
        **reports, "pass": True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freecad-python", type=Path, default=FREECAD_PYTHON)
    parser.add_argument("--output", type=Path, required=True, help="New JSON report path")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("--output already exists.")
    freecad_python = args.freecad_python.absolute()
    if not freecad_python.is_file():
        parser.error("Provide the bundled Python executable via --freecad-python.")
    report = regression(freecad_python, PROTOTYPE / "model")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print("PASS: baseline and regenerated model (41 solids, 6 bidirectional hinges, 4 feet)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
