"""Run reproducible force, gravity, and conservation experiments in MuJoCo."""

import argparse
import csv
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

MODEL = Path(__file__).resolve().parents[1] / "model" / "aether_v1.xml"
BASELINE_DT = 0.002
DURATION = 2.0
GRAVITY = np.array([0.0, 0.0, -9.81])
CASES = ("fall", "inertial", "tumble", "load", "convergence")
CONVERGENCE_STEPS = (0.02, 0.01, 0.005, 0.0025)


def flight_model(path, timestep, gravity, integrator):
    """Fix all moving pieces at CAD neutral and preserve their explicit inertials."""
    xml = ET.parse(path).getroot()
    compiler = xml.find("compiler")
    compiler.set("meshdir", str((path.parent / compiler.get("meshdir")).resolve()))
    compiler.set("fusestatic", "false")
    option = xml.find("option")
    option.set("timestep", str(timestep))
    option.set("integrator", integrator)
    option.set("gravity", " ".join(map(str, gravity)))
    option.set("wind", "0 0 0")
    option.set("density", "0")
    option.set("viscosity", "0")
    xml.remove(xml.find("actuator"))

    world = xml.find("worldbody")
    vehicle = world.find("body[@name='Vehicle']")
    vehicle.find("freejoint").set("align", "false")
    for geom in world.findall("geom"):
        world.remove(geom)
    for body in [vehicle, *vehicle.findall(".//body")]:
        for proxy in body.findall("geom[@class='contact']"):
            body.remove(proxy)
        for joint in body.findall("joint"):
            body.remove(joint)
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    if (model.nq, model.nv, model.nu, model.neq) != (7, 6, 0, 0):
        raise RuntimeError("The flight assembly must have one free joint and no actuators.")
    if model.ngeom != model.nmesh or np.any(model.geom_contype) or np.any(model.geom_conaffinity):
        raise RuntimeError("The flight assembly must contain only collision-disabled visual meshes.")
    if np.any(model.dof_damping) or np.any(model.dof_frictionloss) or np.any(model.dof_armature):
        raise RuntimeError("Unexpected passive effects on the flight degrees of freedom.")
    return model


def inertia_about_com(model, data, root):
    """Use the compiled inertial frames and the parallel-axis theorem."""
    tensor = np.zeros((3, 3))
    center = data.subtree_com[root]
    for i in range(1, model.nbody):
        rotation = data.ximat[i].reshape(3, 3)
        offset = data.xipos[i] - center
        tensor += (rotation * model.body_inertia[i]) @ rotation.T
        tensor += model.body_mass[i] * (offset @ offset * np.eye(3) - np.outer(offset, offset))
    return tensor


def axis_fields(prefix, values, labels="xyz"):
    return {f"{prefix}_{label}": float(value) for label, value in zip(labels, values)}


def integrate(case, path, timestep):
    gravity = GRAVITY if case == "fall" else np.zeros(3)
    integrator = "RK4" if case == "tumble" else "implicitfast"
    model = flight_model(path, timestep, gravity, integrator)
    data = mujoco.MjData(model)
    root = model.body("Vehicle").id
    mass = float(model.body_subtreemass[root])
    data.qpos[:3] = (0, 0, 8)
    mujoco.mj_forward(model, data)
    moments, basis = np.linalg.eigh(inertia_about_com(model, data, root))
    if np.any(moments <= 0) or moments[2] >= moments[0] + moments[1]:
        raise RuntimeError("Vehicle inertia is not positive and physically realizable.")
    if case == "inertial":
        data.qvel[:3] = (1.3, -0.7, 0.25)
    if case == "tumble":
        # Deliberately avoid rotation around any single principal axis.
        data.qvel[3:] = basis @ np.array([1.1, 0.6, 0.8])
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        # Free-joint translation is the CAD origin velocity, so cancel the COM offset velocity.
        data.qvel[:3] -= data.subtree_linvel[root]

    force = np.array([2.0, -1.0, 0.75]) if case == "load" else np.zeros(3)
    axis = basis[:, 2]
    # Eigenvectors are defined only up to sign. Fix it for comparable exported data.
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    torque = 0.6 * axis if case == "load" else np.zeros(3)
    alpha = float(np.dot(torque, axis) / moments[2])
    acceleration = gravity + force / mass

    mujoco.mj_forward(model, data)
    mujoco.mj_subtreeVel(model, data)
    mujoco.mj_energyVel(model, data)
    position0 = data.subtree_com[root].copy()
    velocity0 = data.subtree_linvel[root].copy()
    momentum0 = data.subtree_angmom[root].copy()
    initial_energy = float(data.energy[1])
    q_initial = Rotation.from_quat(data.xquat[root], scalar_first=True)
    velocity = np.empty(6)
    rows = []
    steps = round(DURATION / timestep)
    for step in range(steps + 1):
        t = step * timestep
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        mujoco.mj_energyVel(model, data)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, root, velocity, 0)
        if data.ncon or data.nefc or np.any(data.warning.number) or not np.all(
            np.isfinite(np.r_[data.qpos, data.qvel, data.qacc, data.energy])
        ):
            raise RuntimeError(f"Unconstrained flight check failed in {case} at {t:g} s.")

        position = data.subtree_com[root]
        linear_velocity = data.subtree_linvel[root]
        q = data.xquat[root]
        reference_position = position0 + velocity0 * t + 0.5 * acceleration * t**2
        reference_velocity = velocity0 + acceleration * t
        q_error = omega_error = None
        if case != "tumble":
            reference_rotation = Rotation.from_rotvec(0.5 * alpha * t**2 * axis) * q_initial
            current_rotation = Rotation.from_quat(q, scalar_first=True)
            q_error = float((current_rotation * reference_rotation.inv()).magnitude())
            omega_error = float(np.linalg.norm(velocity[:3] - alpha * t * axis))
        rows.append({
            "time_s": t,
            **axis_fields("com_position_m", position),
            **axis_fields("com_velocity_m_s", linear_velocity),
            **axis_fields("body_quat", q, "wxyz"),
            **axis_fields("omega_world_rad_s", velocity[:3]),
            **axis_fields("test_force_N", force),
            **axis_fields("test_torque_at_com_Nm", torque),
            **axis_fields("gravity_m_s2", gravity),
            **axis_fields("reference_com_position_m", reference_position),
            **axis_fields("reference_com_velocity_m_s", reference_velocity),
            **axis_fields("angular_momentum_world_kg_m2_s", data.subtree_angmom[root]),
            "kinetic_energy_J": float(data.energy[1]),
            "position_error_m": float(np.linalg.norm(position - reference_position)),
            "velocity_error_m_s": float(np.linalg.norm(linear_velocity - reference_velocity)),
            "attitude_error_rad": q_error,
            "angular_velocity_error_rad_s": omega_error,
            "quaternion_norm_error": float(abs(np.linalg.norm(q) - 1)),
            "angular_momentum_relative_drift": None if case != "tumble" else float(
                np.linalg.norm(data.subtree_angmom[root] - momentum0) / np.linalg.norm(momentum0)
            ),
            "energy_relative_drift": None if case != "tumble" else float(
                abs(float(data.energy[1]) - initial_energy) / initial_energy
            ),
        })
        if step < steps:
            data.qfrc_applied[:] = 0
            mujoco.mj_applyFT(model, data, force, torque, position, root, data.qfrc_applied)
            mujoco.mj_step(model, data)

    metrics = {key: max(row[key] for row in rows)
               for key, value in rows[0].items()
               if value is not None and ("error" in key or "drift" in key)}
    bounds = {"quaternion_norm_error": 1e-12}
    if case == "fall":
        bounds.update(position_error_m=10 * DURATION * timestep / 2,
                      velocity_error_m_s=1e-9, attitude_error_rad=1e-9)
    elif case == "inertial":
        bounds.update(position_error_m=1e-9, velocity_error_m_s=1e-9, attitude_error_rad=1e-9)
    elif case == "tumble":
        tolerance = max(1e-10, 5 * timestep**4)
        bounds.update(angular_momentum_relative_drift=tolerance,
                      energy_relative_drift=tolerance,
                      position_error_m=tolerance, velocity_error_m_s=tolerance)
    else:
        bounds.update(position_error_m=0.3 * timestep, velocity_error_m_s=0.04 * timestep,
                      attitude_error_rad=0.4 * timestep, angular_velocity_error_rad_s=1e-6)
    report = {
        "case": case, "integrator": integrator, "timestep_s": timestep, "samples": len(rows),
        "mass_kg": mass, "principal_moments_kg_m2": moments.tolist(),
        "max": metrics, "acceptance_upper_bounds": bounds,
        "pass": all(metrics[key] <= bound for key, bound in bounds.items()),
    }
    return rows, report


def save_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_history(path, case, rows):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    t = [row["time_s"] for row in rows]
    fig, axes = plt.subplots(2, 2, figsize=(9, 6), layout="constrained")
    for ax, prefix, ylabel in (
        (axes[0, 0], "com_position_m", "COM position (m)"),
        (axes[0, 1], "com_velocity_m_s", "COM velocity (m/s)"),
        (axes[1, 0], "omega_world_rad_s", "World angular velocity (rad/s)"),
    ):
        for component in "xyz":
            key = f"{prefix}_{component}"
            line, = ax.plot(t, [row[key] for row in rows], label=component)
            reference_key = f"reference_{key}"
            if reference_key in rows[0]:
                ax.plot(t, [row[reference_key] for row in rows], "--",
                        color=line.get_color(), alpha=0.55)
        ax.legend(ncols=3, loc="best")
        ax.set_ylabel(ylabel)
    ax = axes[1, 1]
    if case == "tumble":
        for key, label in (("angular_momentum_relative_drift", "Angular momentum"),
                           ("energy_relative_drift", "Kinetic energy")):
            ax.plot(t, [row[key] for row in rows], label=label)
        ax.set_ylabel("Relative drift")
        ax.legend(loc="best")
    else:
        ax.plot(t, [row["position_error_m"] for row in rows])
        ax.set_ylabel("COM position error (m)")
    for ax in axes.flat:
        ax.set_xlabel("Time (s)")
        ax.grid(alpha=0.22)
    fig.suptitle(f"Aether Vehicle-V1 · {case.capitalize()}")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def convergence(folder, path, plots):
    trials = []
    for timestep in CONVERGENCE_STEPS:
        rows, report = integrate("load", path, timestep)
        save_csv(folder / f"load_dt_{timestep:g}.csv", rows)
        trials.append(report)
    keys = ("position_error_m", "attitude_error_rad")
    orders = {}
    errors = {}
    for key in keys:
        errors[key] = [trial["max"][key] for trial in trials]
        orders[key] = float(np.polyfit(np.log(CONVERGENCE_STEPS), np.log(errors[key]), 1)[0])
    summary_rows = [{"timestep_s": trial["timestep_s"], **{
        key: trial["max"][key] for key in keys}} for trial in trials]
    save_csv(folder / "convergence.csv", summary_rows)
    if plots:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(9, 3.6), layout="constrained")
        milliseconds = 1000 * np.asarray(CONVERGENCE_STEPS)
        for ax, key, unit in zip(axes, keys, ("mm", "mrad")):
            scaled_errors = 1000 * np.asarray(errors[key])
            ax.loglog(milliseconds, milliseconds * scaled_errors[0] / milliseconds[0],
                      "--", color="#65727a", label="First order")
            ax.loglog(milliseconds, scaled_errors, "o", color="#176a89",
                      label=f"Measured (p={orders[key]:.3f})")
            ax.set(xlabel="Timestep (ms)", ylabel=f"Max {key.split('_')[0]} error ({unit})")
            ax.minorticks_off()
            ax.set_xticks(list(reversed(milliseconds)), ["2.5", "5", "10", "20"])
            ax.set_yticks([0.5, 1, 2, 4], ["0.5", "1", "2", "4"])
            ax.grid(alpha=0.25)
            ax.legend()
        fig.suptitle("Aether Vehicle-V1 · Constant wrench convergence")
        fig.savefig(folder / "convergence.png", dpi=150)
        plt.close(fig)
    return {"trials": trials, "observed_order": orders,
            "pass": all(trial["pass"] for trial in trials)
            and all(np.all(np.diff(values) < 0) for values in errors.values())
            and all(0.85 <= order <= 1.15 for order in orders.values())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("all", *CASES), default="all")
    parser.add_argument("--output", type=Path, required=True, help="New results directory")
    parser.add_argument("--model", type=Path, default=MODEL, help="Vehicle-V1 MJCF path")
    parser.add_argument("--dt", type=float, default=BASELINE_DT,
                        help="Step for single-run cases, in seconds")
    parser.add_argument("--no-plots", action="store_true", help="Write CSV and JSON only")
    args = parser.parse_args()
    model = args.model.resolve()
    if (not 0.0001 <= args.dt <= 0.02
            or not np.isclose(DURATION / args.dt, round(DURATION / args.dt), rtol=0, atol=1e-8)):
        parser.error("--dt must divide 2.0 s exactly and be between 0.0001 and 0.02 s.")
    args.output.mkdir(parents=True, exist_ok=False)
    cases = CASES if args.case == "all" else (args.case,)
    results = {}
    for case in cases:
        if case == "convergence":
            results[case] = convergence(args.output, model, not args.no_plots)
            continue
        rows, results[case] = integrate(case, model, args.dt)
        save_csv(args.output / f"{case}.csv", rows)
        if not args.no_plots:
            plot_history(args.output / f"{case}.png", case, rows)
    passed = all(result["pass"] for result in results.values())
    summary = {"mujoco": mujoco.__version__,
               "model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
               "duration_s": DURATION, "cases": results, "pass": passed}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    for case, result in results.items():
        print(f"{case}: {'PASS' if result['pass'] else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
