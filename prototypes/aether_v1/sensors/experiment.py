"""Record native MuJoCo sensor outputs and independent kinematic references."""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from validation.flight import MODEL, flight_xml, inertia_about_com

CASES = ("rest", "fall", "tumble")
SENSORS = (
    ("accelerometer", "imu_accel", "accel", "m/s²"),
    ("gyro", "imu_gyro", "gyro", "rad/s"),
    ("velocimeter", "ideal_site_vel", "velocity", "m/s"),
)
CHANNELS = [f"{group}_{axis}" for _, _, group, _ in SENSORS for axis in "xyz"]
DT = 0.002
DURATION = 2.0
SITE_BODY_M = np.array([0.035, -0.020, 0.850])
SITE_ROTATION = Rotation.from_euler("xyz", [8, -12, 25], degrees=True)
INITIAL_ROTATION = Rotation.from_euler("xyz", [12, -18, 25], degrees=True)
BASE_POSITION = np.array([0, 0, 25.0])
FALL_VELOCITY = np.array([0.20, -0.10, 0.15])
TUMBLE_OMEGA_BODY = np.array([0.35, -0.55, 0.90])
STATE_COLUMNS = [
    f"{name}_{axis}"
    for name, axes in (
        ("com_position_world_m", "xyz"),
        ("com_velocity_world_m_s", "xyz"),
        ("body_orientation_world", "wxyz"),
        ("site_position_world_m", "xyz"),
        ("site_velocity_world_m_s", "xyz"),
        ("site_acceleration_world_m_s2", "xyz"),
        ("omega_world_rad_s", "xyz"),
    )
    for axis in axes
]


@dataclass
class Recording:
    config: dict
    scene_xml: str
    stamps: np.ndarray
    truth: np.ndarray
    kinematics: np.ndarray
    raw: np.ndarray
    measured: np.ndarray


def numbers(vector):
    return " ".join(f"{value:.17g}" for value in vector)


def build_scene(case, model_path, gravity):
    xml = flight_xml(model_path, DT, gravity, "RK4")
    vehicle = xml.find("./worldbody/body[@name='Vehicle']")
    vehicle.set("pos", numbers(BASE_POSITION))
    vehicle.set("quat", numbers(INITIAL_ROTATION.as_quat(scalar_first=True)))
    if case == "rest":
        # Fix the fixture to world without soft constraints or a contact solver.
        vehicle.remove(vehicle.find("freejoint"))
    site = ET.SubElement(vehicle, "site", name="measurement_site", type="sphere",
                         size="0.006", rgba="0 0 0 0", pos=numbers(SITE_BODY_M),
                         quat=numbers(SITE_ROTATION.as_quat(scalar_first=True)))
    sensors = ET.SubElement(xml, "sensor")
    for tag, name, _, _ in SENSORS:
        ET.SubElement(sensors, tag, name=name, site=site.get("name"))
    xml_text = ET.tostring(xml, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml_text)
    expected_dof = 0 if case == "rest" else 6
    if (model.nv, model.nu, model.neq, model.nsensordata) != (expected_dof, 0, 0, 9):
        raise RuntimeError("Unexpected degrees of freedom, controls, or sensor dimensions.")
    if np.any(model.geom_contype) or np.any(model.geom_conaffinity):
        raise RuntimeError("The sensor scenario must be contact-free.")
    xml.find("compiler").set("meshdir", "meshes")
    ET.indent(xml)
    return model, ET.tostring(xml, encoding="unicode") + "\n"


def run_case(case, model_path=MODEL, noise_std=(0.0, 0.0, 0.0), seed=31):
    if case not in CASES or len(noise_std) != len(SENSORS) or not np.all(
        np.isfinite(noise_std) & (np.array(noise_std) >= 0)
    ) or seed < 0:
        raise ValueError("Invalid case, noise standard deviation, or seed.")
    gravity = np.zeros(3) if case == "tumble" else np.array([0.0, 0.0, -9.81])
    model, xml_text = build_scene(case, model_path.resolve(), gravity)
    state = mujoco.MjData(model)
    root = model.body("Vehicle").id
    initial_rotation = INITIAL_ROTATION.as_matrix()
    if case != "rest":
        state.qpos[:3] = BASE_POSITION
        state.qpos[3:7] = INITIAL_ROTATION.as_quat(scalar_first=True)
        state.qvel[:3] = FALL_VELOCITY if case == "fall" else 0
        if case == "tumble":
            state.qvel[3:] = TUMBLE_OMEGA_BODY
    mujoco.mj_forward(model, state)
    body_com = initial_rotation.T @ (state.subtree_com[root] - state.xpos[root])
    if case == "tumble":
        state.qvel[:3] = -np.cross(initial_rotation @ TUMBLE_OMEGA_BODY,
                                   initial_rotation @ body_com)
    initial_world_inertia = inertia_about_com(model, state, root)
    initial_body_inertia = initial_rotation.T @ initial_world_inertia @ initial_rotation
    site_rotation_body = SITE_ROTATION.as_matrix()
    steps = round(DURATION / DT)
    times = np.empty(steps + 1)
    raw = np.empty((steps + 1, len(CHANNELS)))
    truth = np.empty_like(raw)
    kinematics = np.empty((steps + 1, len(STATE_COLUMNS)))

    for i in range(steps + 1):
        # Align all position-, velocity-, and acceleration-stage outputs to state.time.
        mujoco.mj_forward(model, state)
        if (state.ncon or state.nefc or np.any(state.warning.number)
                or not np.all(np.isfinite(state.sensordata))):
            raise RuntimeError(f"Unexpected constraint or solver/sensor failure at step {i}.")
        times[i] = state.time
        q_world = state.xquat[root]
        rotation = Rotation.from_quat(q_world, scalar_first=True).as_matrix()
        world_from_sensor = rotation @ site_rotation_body
        origin = state.xpos[root]
        world_com_offset = rotation @ body_com
        site_offset = rotation @ SITE_BODY_M
        radius = site_offset - world_com_offset
        omega = rotation @ state.qvel[3:] if case != "rest" else np.zeros(3)
        origin_velocity = state.qvel[:3] if case != "rest" else np.zeros(3)
        com_velocity = origin_velocity + np.cross(omega, world_com_offset)
        site_velocity = com_velocity + np.cross(omega, radius)
        alpha = np.zeros(3)
        if case == "tumble":
            world_inertia = rotation @ initial_body_inertia @ rotation.T
            # Instantaneous torque-free Euler identity, used only as a measurement reference.
            alpha = np.linalg.solve(world_inertia, -np.cross(omega, world_inertia @ omega))
        com_acceleration = gravity if case == "fall" else np.zeros(3)
        site_acceleration = com_acceleration + np.cross(alpha, radius) + np.cross(
            omega, np.cross(omega, radius)
        )
        truth[i] = np.concatenate([
            world_from_sensor.T @ (site_acceleration - gravity),
            world_from_sensor.T @ omega,
            world_from_sensor.T @ site_velocity,
        ])
        raw[i] = np.concatenate([state.sensor(name).data for _, name, _, _ in SENSORS])
        kinematics[i] = np.concatenate([
            origin + world_com_offset, com_velocity, q_world, origin + site_offset,
            site_velocity, site_acceleration, omega,
        ])
        if i < steps:
            if case == "rest":
                # MuJoCo has zero mechanical DOFs in the world-fixed fixture.
                state.time = (i + 1) * DT
            else:
                mujoco.mj_step(model, state)

    sigma = np.repeat(np.asarray(noise_std, dtype=float), 3)
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, CASES.index(case)])))
    measured = raw + rng.normal(0.0, sigma, raw.shape) if np.any(sigma) else raw.copy()
    config = {
        "schema_version": 1, "case": case, "mujoco_version": mujoco.__version__,
        "numpy_version": np.__version__, "model_sha256": hashlib.sha256(
            model_path.read_bytes()
        ).hexdigest(),
        "scene_sha256": hashlib.sha256(xml_text.encode()).hexdigest(),
        "sample_period_s": DT, "duration_s": DURATION, "sample_count": len(times),
        "integrator": "world-fixed" if case == "rest" else "RK4",
        "time_origin": "simulation start; seconds", "gravity_world_m_s2": gravity.tolist(),
        "initial_body_origin_world_m": BASE_POSITION.tolist(),
        "initial_body_quaternion_world_wxyz": INITIAL_ROTATION.as_quat(scalar_first=True).tolist(),
        "initial_com_velocity_world_m_s": kinematics[0, 3:6].tolist(),
        "initial_body_omega_rad_s": TUMBLE_OMEGA_BODY.tolist() if case == "tumble" else [0, 0, 0],
        "vehicle_concept_mass_kg": float(model.body_subtreemass[root]),
        "frames": {
            "world": "+Z up, neutral CAD axes",
            "body": "Vehicle CAD frame, +Z toward nose; orientation is body to world",
            "site_origin_body_m": SITE_BODY_M.tolist(),
            "site_to_body_quaternion_wxyz": SITE_ROTATION.as_quat(scalar_first=True).tolist(),
            "measurements": "site frame; accelerometer reports proper acceleration a_site minus g",
        },
        "noise": {"distribution": "independent zero-mean Gaussian after native sampling",
                  "generator": "NumPy PCG64", "seed": seed,
                  "case_seed_index": CASES.index(case), "sigma_by_group": {
                      group: float(std) for (_, _, group, _), std in zip(SENSORS, noise_std)
                  }},
        "channels": [
            {"group": group, "sensor": name, "type": tag, "unit": unit,
             "columns": [f"{group}_{axis}" for axis in "xyz"]}
            for tag, name, group, unit in SENSORS
        ],
    }
    return Recording(config, xml_text, times, truth, kinematics, raw, measured)


def record(folder, recording):
    folder.mkdir(parents=True, exist_ok=False)
    for name, columns, values in (
        ("truth", [*CHANNELS, *STATE_COLUMNS],
         np.column_stack([recording.truth, recording.kinematics])),
        ("raw", CHANNELS, recording.raw),
        ("measurements", CHANNELS, recording.measured),
    ):
        np.savetxt(folder / f"{name}.csv", np.column_stack([recording.stamps, values]), delimiter=",",
                   header=",".join(["time_s", *columns]), comments="", fmt="%.17g")
    (folder / "config.json").write_text(json.dumps(recording.config, indent=2) + "\n")
    (folder / "scene.xml").write_text(recording.scene_xml)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("all", *CASES), default="all")
    parser.add_argument("--output", type=Path, required=True, help="New output directory")
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--accel-std", type=float, default=0, help="Noise sigma in m/s² per axis")
    parser.add_argument("--gyro-std", type=float, default=0, help="Noise sigma in rad/s per axis")
    parser.add_argument("--velocity-std", type=float, default=0, help="Noise sigma in m/s per axis")
    args = parser.parse_args()
    noise = (args.accel_std, args.gyro_std, args.velocity_std)
    if args.seed < 0 or not np.all(np.isfinite(noise)) or min(noise) < 0:
        parser.error("Noise standard deviations must be finite and nonnegative, and seed >= 0.")
    args.output.mkdir(parents=True, exist_ok=False)
    for case in CASES if args.case == "all" else (args.case,):
        recording = run_case(case, args.model, noise, args.seed)
        record(args.output / case, recording)
        print(f"Recorded {case}: {recording.config['sample_count']} synchronized samples")


if __name__ == "__main__":
    main()
