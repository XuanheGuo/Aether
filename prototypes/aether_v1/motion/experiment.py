"""Ground departure and torque-free motion of the frozen Vehicle-V1 assembly."""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from prototypes.aether_v1.validation.flight import MODEL, axis_fields, flight_xml, inertia_about_com, save_csv

CASES = ("vertical", "axis", "tumble", "inertia_variant")
GRAVITY = np.array([0., 0., -9.81])
DT = .002
BOOST = 1.
CUTOFF = 2.
AXIAL_INCREMENT = .08
TILT_DEG = 8.
OMEGA_BODY = [.28, .18, .48]


def build(case, dt):
    vertical = case == "vertical"
    model_xml = flight_xml(MODEL, dt, GRAVITY if vertical else np.zeros(3),
                           "implicitfast" if vertical else "RK4")
    world = model_xml.find("worldbody")
    vehicle = world.find("body[@name='Vehicle']")
    if vertical:
        # Bring back the exact frozen support plane and collision proxies.
        source = ET.parse(MODEL).find("worldbody")
        world.append(source.find("geom[@name='floor']"))
        for body in [vehicle, *vehicle.findall(".//body")]:
            original = source.find(f".//body[@name='{body.get('name')}']")
            body.extend(original.findall("geom[@class='contact']"))
    elif case == "inertia_variant":
        inertial = vehicle.find("inertial")
        values = list(map(float, inertial.get("fullinertia").split()))
        values[2] += AXIAL_INCREMENT
        inertial.set("fullinertia", " ".join(map(str, values)))
    model_xml.set("model", f"Aether V1 — {case}")
    model = mujoco.MjModel.from_xml_string(ET.tostring(model_xml, encoding="unicode"))
    if (model.nq, model.nv, model.nu, model.neq) != (7, 6, 0, 0):
        raise ValueError("Expected one free joint with the CAD mechanism locked at neutral.")
    if any(np.any(v) for v in (model.dof_damping, model.dof_frictionloss, model.dof_armature)):
        raise ValueError("Unexpected passive dynamics on the free joint.")
    return model


def contacts(model, data):
    force = np.zeros(3)
    record = set()
    reaction = np.empty(6)
    floor = model.geom("floor").id
    for i in range(data.ncon):
        contact = data.contact[i]
        if floor not in contact.geom:
            raise ValueError("Unexpected contact outside the test plane.")
        first, second = map(int, contact.geom)
        mujoco.mj_contactForce(model, data, i, reaction)
        sign = 1 if first == floor else -1
        force += sign * (contact.frame.reshape(3, 3).T @ reaction[:3])
        if contact.efc_address >= 0:
            record.add(model.geom(second if first == floor else first).name)
    return force, "|".join(sorted(record))


def load_factor(t):
    if BOOST <= t < CUTOFF:
        return 1.25
    if .4 <= t < BOOST:
        return .75
    return 0.


def run(case, dt=DT, on_frame=None):
    model = build(case, dt)
    data = mujoco.MjData(model)
    root = model.body("Vehicle").id
    mass = float(model.body_subtreemass[root])
    vertical = case == "vertical"
    mujoco.mj_forward(model, data)
    moments, basis = np.linalg.eigh(inertia_about_com(model, data, root))
    if moments[0] <= 0 or moments[2] >= moments[0] + moments[1]:
        raise ValueError("Assembly has nonphysical principal moments.")
    initial_attitude = Rotation.identity()
    if not vertical:
        initial_attitude = Rotation.from_euler("x", TILT_DEG, degrees=True)
        data.qpos[2] = 1.2
        data.qpos[3:] = initial_attitude.as_quat(scalar_first=True)
        axis = basis[:, 0].copy()
        if axis[2] < 0:
            axis *= -1
        initial_omega = .55 * axis if case == "axis" else OMEGA_BODY
        data.qvel[3:] = initial_omega
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        data.qvel[:3] -= data.subtree_linvel[root]

    duration = 3.5 if vertical else 4.
    steps = round(duration / dt)
    gravity = model.opt.gravity * mass
    rows = []
    twist = np.empty(6)
    force = np.zeros(3)
    torque = np.zeros(3)
    for step in range(steps + 1):
        t = step * dt
        data.qfrc_applied[:] = 0.
        if vertical:
            force[2] = load_factor(t) * abs(gravity[2])
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            mujoco.mj_applyFT(model, data, force, torque, data.subtree_com[root], root, data.qfrc_applied)
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        mujoco.mj_energyVel(model, data)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, root, twist, 0)
        if np.any(data.warning.number) or not np.isfinite(np.r_[data.qpos, data.qvel, data.qacc]).all():
            raise ValueError(f"Invalid native state in {case} at {t} s.")
        reaction, active = contacts(model, data) if vertical else (np.zeros(3), "")
        external = gravity + force + reaction
        rows.append({
            "time_s": t,
            **axis_fields("com_position_m", data.subtree_com[root]),
            **axis_fields("base_position_m", data.qpos[:3]),
            **axis_fields("com_velocity_m_s", data.subtree_linvel[root]),
            **axis_fields("com_accel_force_m_s2", external / mass),
            **axis_fields("body_quat", data.xquat[root], "wxyz"),
            **axis_fields("omega_world_rad_s", twist[:3]),
            **axis_fields("omega_body_rad_s", data.qvel[3:]),
            **axis_fields("angular_momentum_world_kg_m2_s", data.subtree_angmom[root]),
            **axis_fields("applied_test_force_N", force),
            **axis_fields("gravity_force_N", gravity),
            **axis_fields("contact_force_N", reaction),
            **axis_fields("total_external_force_N", external),
            "kinetic_energy_J": float(data.energy[1]),
            "contact_points": data.ncon, "contact_geoms": active,
        })
        if on_frame:
            on_frame(step, model, data, rows[-1])
        if step < steps:
            mujoco.mj_step(model, data)

    def extract(prefix, axes="xyz"):
        return np.array([[row[f"{prefix}_{axis}"] for axis in axes] for row in rows])

    velocity = extract("com_velocity_m_s")
    acceleration = np.gradient(velocity, dt, axis=0, edge_order=2)
    for row, value in zip(rows, acceleration, strict=True):
        row.update(axis_fields("com_accel_velocity_fd_m_s2", value))
    report = {"case": case, "mass_kg": mass, "timestep_s": dt, "duration_s": duration,
              "samples": len(rows), "integrator": "implicitfast" if vertical else "RK4",
              "principal_moments_kg_m2": moments.tolist()}
    q = Rotation.from_quat(extract("body_quat", "wxyz"), scalar_first=True)
    quat_error = float(np.max(abs(np.linalg.norm(extract("body_quat", "wxyz"), axis=1) - 1)))
    report["max_quaternion_norm_error"] = quat_error
    times = np.arange(steps + 1) * dt
    if vertical:
        report.update(check_vertical(rows, times, extract("com_position_m"), velocity, acceleration, mass))
    else:
        momentum = extract("angular_momentum_world_kg_m2_s")
        energy = np.asarray([row["kinetic_energy_J"] for row in rows])
        h_drift = float(np.linalg.norm(momentum - momentum[0], axis=1).max() / np.linalg.norm(momentum[0]))
        e_drift = float(np.max(abs(energy / energy[0] - 1)))
        report.update({
            "initial_tilt_deg": TILT_DEG, "initial_omega_body_rad_s": list(initial_omega),
            "initial_angular_momentum_world_kg_m2_s": momentum[0].tolist(), "initial_kinetic_energy_J": float(energy[0]),
            "angular_momentum_max_relative_drift": h_drift, "kinetic_energy_max_relative_drift": e_drift,
            "max_com_speed_m_s": float(np.linalg.norm(velocity, axis=1).max()),
            "initial_to_final_angle_deg": float((q[-1] * q[0].inv()).magnitude() * 180 / np.pi),
            "body_axial_inertia_increment_kg_m2": AXIAL_INCREMENT if case == "inertia_variant" else 0.,
        })
        for i, row in enumerate(rows):
            row["angular_momentum_relative_drift"] = np.linalg.norm(momentum[i] - momentum[0]) / np.linalg.norm(momentum[0])
            row["kinetic_energy_relative_drift"] = abs(energy[i] / energy[0] - 1)
        report["pass"] = (max(h_drift, e_drift) < 2e-7 and report["max_com_speed_m_s"] < 1e-7
                          and all(row["contact_points"] == 0 for row in rows))
        if case == "axis":
            truth = initial_attitude * Rotation.from_rotvec(times[:, None] * initial_omega)
            error = (q * truth.inv()).magnitude()
            for row, value in zip(rows, error, strict=True):
                row["attitude_reference_error_rad"] = value
            report["principal_axis_max_attitude_error_rad"] = float(error.max())
            report["pass"] &= bool(error.max() < 1e-7)
    report["pass"] &= quat_error < 1e-12
    return rows, report


def check_vertical(rows, t, position, velocity, accel, mass):
    active = np.array([bool(row["contact_geoms"]) for row in rows])
    flying = np.flatnonzero((t >= BOOST) & ~active)
    if not len(flying):
        raise ValueError("Vehicle never left the test plane.")
    first = flying[0]
    landed = np.flatnonzero((t > CUTOFF) & active)
    if not len(landed) or landed[0] <= first:
        raise ValueError("No recontact after the load was removed.")
    stop = landed[0]
    free = (np.arange(len(t)) >= first) & (np.arange(len(t)) < stop)
    dt = t[1] - t[0]
    elapsed = t - t[first]
    ballistic = np.maximum(t - CUTOFF, 0)
    expected_a = np.where(t < CUTOFF, .25, -1.) * abs(GRAVITY[2])
    real_z = position[:, 2]
    real_vz = velocity[:, 2]
    z_ref = real_z[first] + real_vz[first] * elapsed + .5 * abs(GRAVITY[2]) * (.25 * elapsed**2 - 1.25 * ballistic**2)
    v_ref = real_vz[first] + abs(GRAVITY[2]) * (.25 * elapsed - 1.25 * ballistic)
    stable = free & (t > t[first] + 2 * dt) & (abs(t - CUTOFF) > 2 * dt) & (t < t[stop] - 2 * dt)
    sub = (.7 <= t) & (t < .97)
    support = np.mean([row["contact_force_N_z"] for row, keep in zip(rows, sub) if keep])
    for i, row in enumerate(rows):
        row["height_above_initial_com_m"] = real_z[i] - real_z[0]
        row["reference_com_z_m"] = z_ref[i] if free[i] else None
        row["reference_com_vz_m_s"] = v_ref[i] if free[i] else None
        row["reference_com_az_m_s2"] = expected_a[i] if free[i] else None
    zerr = float(np.max(abs(real_z[free] - z_ref[free])))
    verr = float(np.max(abs(real_vz[free] - v_ref[free])))
    aerr = float(np.max(abs(accel[stable, 2] - expected_a[stable])))
    report = {
        "gravity_m_s2": GRAVITY[2], "test_force_subweight_N": .75 * mass * abs(GRAVITY[2]),
        "test_force_boost_N": 1.25 * mass * abs(GRAVITY[2]),
        "subweight_mean_support_N": float(support),
        "liftoff_s": float(t[first]), "recontact_s": float(t[stop]),
        "peak_com_rise_m": float(np.max(real_z - real_z[0])),
        "max_free_z_error_m": zerr, "max_free_vz_error_m_s": verr,
        "max_free_fd_accel_error_m_s2": aerr,
        "boost_expected_accel_m_s2": .25 * abs(GRAVITY[2]),
        "boost_mean_fd_accel_m_s2": float(np.mean(accel[stable & (t < CUTOFF), 2])),
        "ballistic_mean_fd_accel_m_s2": float(np.mean(accel[stable & (t > CUTOFF), 2])),
        "pass": bool(active[sub].all() and (~active[free]).all() and active[-1]
                     and t[first] <= BOOST + .05 and np.isclose(support, .25 * mass * abs(GRAVITY[2]), atol=.15)
                     and zerr < 7 * dt and verr < 1e-6 and aerr < 1e-5),
    }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--view", choices=CASES)
    parser.add_argument("--auto-close", action="store_true", help="Close the test Viewer after its run.")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    history, metrics = {}, {}
    for case in CASES:
        if case == args.view:
            from prototypes.aether_v1.motion.view import view
            rows, report = view(case, args.auto_close)
        else:
            rows, report = run(case)
        history[case], metrics[case] = rows, report
        save_csv(args.output / f"{case}.csv", rows)
    from prototypes.aether_v1.motion.plots import plot

    fine_rows, alternate = run("vertical", DT / 2)
    save_csv(args.output / "vertical_dt_1ms.csv", fine_rows)
    q = [Rotation.from_quat([history[c][-1][f"body_quat_{a}"] for a in "wxyz"], scalar_first=True)
         for c in ("tumble", "inertia_variant")]
    delta = float((q[1] * q[0].inv()).magnitude() * 180 / np.pi)
    ratio = metrics["vertical"]["max_free_z_error_m"] / alternate["max_free_z_error_m"]
    summary = {
        "mujoco_version": mujoco.__version__, "frozen_mjcf_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        "cases": metrics, "vertical_half_timestep": alternate, "vertical_position_error_refinement_ratio": ratio,
        "tumble_final_attitude_difference_deg": delta,
        "pass": all(item["pass"] for item in metrics.values()) and alternate["pass"] and delta > 1 and 1.8 < ratio < 2.2,
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    plot(args.output, history)
    print(f"Passed: {summary['pass']} · {args.output}")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
