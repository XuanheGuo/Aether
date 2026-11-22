"""Validate the compiled MuJoCo model against the FreeCAD motion tests."""

import json
from pathlib import Path

import mujoco
import numpy as np


def check(folder):
    manifest = json.loads((folder / "assembly.json").read_text())
    cad_results = json.loads((folder / "cad_checks.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(folder / "aether_v1.xml"))
    data = mujoco.MjData(model)
    assert model.nmesh == len(manifest["parts"])
    assert model.njnt == len(manifest["joints"]) + 1
    mass = model.body_mass[1:]
    inertia = np.sort(model.body_inertia[1:], axis=1)
    assert np.all(np.isfinite(mass)) and np.all(mass > 0)
    assert np.all(np.isfinite(inertia)) and np.all(inertia > 0)
    assert np.all(inertia[:, 2] < inertia[:, 0] + inertia[:, 1])
    for name, attributes in manifest["parts"].items():
        geom = model.geom(name)
        assert geom.contype[0] == geom.conaffinity[0] == 0
        assert geom.bodyid[0] == model.body(attributes["body"]).id

    position_error = 0.0
    quaternion_error = 0.0
    for pose in cad_results["poses"]:
        for name, angle in pose["angles_degrees"].items():
            data.qpos[model.joint(name).qposadr[0]] = np.deg2rad(angle)
        mujoco.mj_forward(model, data)
        for name, expected in pose["global"].items():
            body = data.body(name)
            reference_q = np.array(expected["quaternion_wxyz"])
            position_error = max(position_error, np.linalg.norm(
                body.xpos - np.array(expected["position_mm"]) / 1000))
            quaternion_error = max(quaternion_error, min(np.linalg.norm(body.xquat - reference_q),
                                                         np.linalg.norm(body.xquat + reference_q)))
    assert position_error < 1e-7 and quaternion_error < 1e-7, (position_error, quaternion_error)

    mujoco.mj_resetData(model, data)
    contact_feet = set()
    setpoints = {}
    for i, (name, attributes) in enumerate(manifest["joints"].items()):
        target = np.deg2rad(attributes["range_degrees"][1]) * 0.7 * (-1) ** i
        setpoints[name] = target
        data.ctrl[model.actuator(name + "_target").id] = target
    for _ in range(2500):
        mujoco.mj_step(model, data)
        assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
        for contact in data.contact:
            for geom in contact.geom:
                name = model.geom(geom).name
                if name.startswith("Foot"):
                    contact_feet.add(name)
    assert not any(warning.number != 0 for warning in data.warning)
    assert len(contact_feet) == 4, contact_feet
    assert abs(data.qpos[2]) < 0.01, data.qpos[:3]
    tracking_errors = {
        name: abs(np.rad2deg(data.qpos[model.joint(name).qposadr[0]] - target))
        for name, target in setpoints.items()
    }
    # The passive nozzle weight produces a finite steady-state error.
    assert max(tracking_errors.values()) < 6, tracking_errors
    report = {"mujoco": mujoco.__version__, "meshes": model.nmesh, "hinges": model.njnt - 1,
              "concept_mass_kg": round(sum(mass), 5),
              "cad_pose_comparisons": len(cad_results["poses"]),
              "max_position_error_m": position_error,
              "max_quaternion_component_norm_error": quaternion_error,
              "elapsed_physics_s": round(data.time, 3), "ground_contact_feet": sorted(contact_feet),
              "max_actuator_error_degrees": round(max(tracking_errors.values()), 4)}
    (folder / "sim_checks.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    print(json.dumps(check(Path(__file__).resolve().parent / "model"), indent=2))
