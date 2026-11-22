"""Verify the recorded closed loop from raw MAVLink frames, MuJoCo and PX4 ULog."""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from pymavlink.dialects.v20 import common as mav
from pyulog import ULog

from prototypes.iris_closed_loop.simulator import (
    ACTUATOR,
    DT_US,
    MAG_MASK,
    MODEL,
    START_US,
    motor_step,
)
from prototypes.mujoco_sitl.bridge import ENU_TO_NED, geodetic
from prototypes.mujoco_sitl.verify import clock_match, read_frames, values


def vector(rows, name, count):
    return values(rows, [f"{name}_{i}" for i in range(count)])


def rmse(errors):
    return float(np.sqrt(np.mean(np.asarray(errors) ** 2)))


def angle(a, b):
    cosine = np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
    return np.rad2deg(2 * np.arccos(np.clip(abs(cosine), 0, 1)))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(folder):
    truth = np.genfromtxt(folder / "truth.csv", delimiter=",", names=True)
    groups, counts, bad, gaps = read_frames(folder / "wire.jsonl")
    simulator = json.loads((folder / "simulator.json").read_text())
    run = json.loads((folder / "run.json").read_text())
    stamps = truth["time_usec"].astype(np.int64)
    require(len(stamps) == simulator["steps"] and stamps[0] == START_US
            and np.all(np.diff(stamps) == DT_US), "Invalid native clock.")
    require(np.isfinite(truth.view(np.float64)).all(), "Nonfinite native data.")
    require(not bad and not any(gaps.values()), "Invalid frame or missing wire sequence.")
    require(run["stage"] == "completed" and run["px4_exit_code"] == 0, "Unclean runner exit.")
    return truth, stamps, groups, counts, simulator, run


def check_exchange(truth, stamps, groups):
    tx = groups["tx"]
    sensor = {msg.time_usec: msg for _, msg in tx["HIL_SENSOR"]}
    returns = {msg.time_usec: msg for _, msg in groups["rx"][ACTUATOR]}
    matched = truth["matched"].astype(bool)
    armed = truth["armed"].astype(bool)
    force = vector(truth, "actuator_force", 4)
    controls, thrust = vector(truth, "control", 4), vector(truth, "thrust", 4)
    require(np.all(~armed | matched) and np.array_equal(
        truth["return_time_usec"][matched], stamps[matched]
    ), "Armed tick without same-tick return.")
    require(np.all(truth["lockstep_flag"][matched] == 1)
            and np.allclose(thrust, force, atol=1e-12, rtol=0), "Lockstep flag or native force differs.")
    for i in np.flatnonzero(matched):
        msg = returns[stamps[i]]
        require(msg.flags & 1 and bool(msg.mode & mav.MAV_MODE_FLAG_SAFETY_ARMED) == armed[i]
                and np.array_equal(msg.controls[:4], controls[i]), "Capture and applied controls differ.")
    speed = np.zeros(4)
    speed_log = vector(truth, "rotor_speed", 4)
    for i, command in enumerate(controls):
        speed, expected = motor_step(speed, command, armed[i], DT_US / 1e6)
        require(np.allclose(expected, force[i], atol=1e-12, rtol=0)
                and np.allclose(speed, speed_log[i], atol=1e-12, rtol=0), "Motor transfer differs.")
    sensor_stamps = np.array(list(sensor))
    require(np.array_equal(sensor_stamps, np.arange(stamps[0], sensor_stamps[-1] + DT_US, DT_US))
            and list(sensor)[-1] in (stamps[-1], stamps[-1] + DT_US), "HIL_SENSOR clock differs.")
    fields = ("xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag")
    measurements = np.array([[getattr(sensor[t], field) for field in fields] for t in stamps])
    native = np.column_stack([vector(truth, key, 3) for key in ("accel_frd", "gyro_frd", "mag_sent_frd")])
    require(np.array_equal(measurements, native.astype(np.float32)), "FRD sensor values differ on wire.")
    for kind in ("HIL_GPS", "HIL_STATE_QUATERNION"):
        require(np.array_equal([m.time_usec for _, m in tx[kind]],
                               sensor_stamps[::5][:len(tx[kind])])
                and len(tx[kind]) in (len(sensor_stamps[::5]), len(sensor_stamps[::5]) - 1),
                f"{kind} clock differs.")
    p = vector(truth, "position_enu", 3)
    displacement = (p - p[0]) @ ENU_TO_NED.T
    v_ned = vector(truth, "velocity_ned", 3)
    q_ned = vector(truth, "q_ned", 4).astype(np.float32)
    for _, m in tx["HIL_GPS"]:
        i = (m.time_usec - START_US) // DT_US
        if i == len(stamps):
            continue
        require(m.fix_type == 3 and (m.lat, m.lon, m.alt) == geodetic(displacement[i])
                and np.array_equal(np.rint(100 * v_ned[i]), [m.vn, m.ve, m.vd]),
                "GNSS fix, geodetic representation, or wire units differ.")
    for _, m in tx["HIL_STATE_QUATERNION"]:
        i = (m.time_usec - START_US) // DT_US
        if i == len(stamps):
            continue
        require(np.array_equal(q_ned[i], m.attitude_quaternion)
                and np.array_equal(np.rint(100 * v_ned[i]), [m.vx, m.vy, m.vz]),
                "State quaternion or cm/s wire units differ.")
    match_latency = truth["return_latency_ms"][matched]
    require(len(match_latency) and np.min(match_latency) >= 0, "Invalid wire latency.")
    mag_stamps = np.array([t for t, message in sensor.items() if message.fields_updated & MAG_MASK])
    require(np.isin(np.diff(mag_stamps), (8000, 12000)).all(), "Magnetometer cadence differs.")
    return {
        "integrated_ticks": len(stamps), "matched_ticks": int(matched.sum()),
        "armed_ticks": int(armed.sum()), "unmatched_disarmed_ticks": int((~matched).sum()),
        "host_imu_to_actuator_ms": dict(zip(("median", "p95", "max"),
                                           np.percentile(match_latency, [50, 95, 100]).tolist())),
        "sensor_hz": {"imu": 1e6 / DT_US, "mag_mean": float(1e6 / np.diff(mag_stamps).mean()),
                      "state": 1e6 / (5 * DT_US), "gnss": 1e6 / (5 * DT_US)},
    }


def measure_flight(truth, stamps, groups, ulog):
    def get(topic):
        return ulog.get_dataset(topic).data

    status = get("vehicle_status")
    changes = np.r_[True, (np.diff(status["nav_state"]) != 0) | (np.diff(status["arming_state"]) != 0)]
    modes = status["nav_state"][changes].tolist()
    arming = status["arming_state"][changes].tolist()
    require(modes == [4, 17, 4, 18, 4] and arming == [1, 2, 2, 2, 1]
            and not status["failsafe"].any(), "Unexpected PX4 flight lifecycle.")
    require(not ulog.dropouts, "PX4 ULog reported dropped records.")
    start, hold_start, land_start, disarm = status["timestamp"][changes][1:]
    land = get("vehicle_land_detected")
    require(np.any(~land["landed"] & (land["timestamp"] > start))
            and bool(land["landed"][-1]) and land["timestamp"][-1] < disarm
            and truth["contacts"][-1] > 0, "Landing not corroborated by contact and PX4.")
    require(truth["height_m"].max() > 1.5 and abs(truth["height_m"][-1]) < .02,
            "MuJoCo did not leave and return to the ground.")
    pose = get("vehicle_local_position")
    pose_t, pose_i = clock_match(pose, "timestamp_sample", stamps)
    hover = (pose_t >= hold_start) & (pose_t < land_start)
    require(hover.any() and all(pose[name][hover].all()
                                for name in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")),
            "EKF position or velocity unavailable in Hold.")
    ned = (vector(truth, "position_enu", 3) - vector(truth, "position_enu", 3)[0]) @ ENU_TO_NED.T
    estimate = values(pose, ("x", "y", "z"))
    # Resolve the independent EKF position origin from pre-arm samples only.
    offset = np.median((estimate - ned[pose_i])[pose_t < start], axis=0)
    require(np.isfinite(offset).all(), "Pre-arm EKF origin unavailable.")
    est_error = estimate[hover] - (ned[pose_i[hover]] + offset)
    vel_error = values(pose, ("vx", "vy", "vz"))[hover] - vector(truth, "velocity_ned", 3)[pose_i[hover]]

    def sample(target, source, data):
        indices = np.searchsorted(source, target, side="right") - 1
        require(np.all(indices >= 0), "Missing setpoint samples.")
        return data[indices]

    target = get("vehicle_local_position_setpoint")
    z_target = sample(pose_t[hover], target["timestamp"], target["z"])
    require(np.isfinite(z_target).all(), "Hold has no altitude setpoint.")
    att = get("vehicle_attitude")
    att_t, att_i = clock_match(att, "timestamp_sample", stamps)
    att_hover = (att_t >= hold_start) & (att_t < land_start)
    q = values(att, [f"q[{i}]" for i in range(4)])[att_hover]
    q_target = get("vehicle_attitude_setpoint")
    q_d = sample(att_t[att_hover], q_target["timestamp"],
                 values(q_target, [f"q_d[{i}]" for i in range(4)]))
    att_error = angle(q, vector(truth, "q_ned", 4)[att_i[att_hover]])
    att_track = angle(q, q_d)
    sensor_metrics = check_ulog_sensors(truth, stamps, groups, get)
    hold_i = np.flatnonzero((stamps >= hold_start) & (stamps < land_start) & (truth["contacts"] == 0))
    replay = replay_step(truth, hold_i[len(hold_i) // 2])
    altitude_tracking = rmse(estimate[hover, 2] - z_target)
    attitude_tracking = rmse(att_track)
    hold_s = float((land_start - hold_start) / 1e6)
    require(hold_s > 4 and altitude_tracking < .4 and attitude_tracking < 5,
            "Flight did not meet Hold duration and tracking tolerances.")
    metrics = {
        "px4_nav_states": modes, "px4_arm_states": arming, "ulog_dropouts": len(ulog.dropouts),
        "peak_height_m": float(truth["height_m"].max()), "final_height_m": float(truth["height_m"][-1]),
        "hold_s": hold_s,
        "hold_truth_mean_height_m": float(truth["height_m"][hold_i].mean()),
        "hold_mean_collective_body_thrust_n": float(vector(truth, "actuator_force", 4)[hold_i].sum(axis=1).mean()),
        "hold_estimator_position_rmse_ned_m": [rmse(axis) for axis in est_error.T],
        "hold_estimator_velocity_rmse_ned_m_s": [rmse(axis) for axis in vel_error.T],
        "hold_estimator_attitude_rmse_deg": rmse(att_error),
        "hold_control_altitude_rmse_m": altitude_tracking,
        "hold_control_attitude_rmse_deg": attitude_tracking,
        "motor_force_counterfactual": replay, "ulog_sensor_validation": sensor_metrics,
    }
    series = {
        "pose_s": (pose_t - START_US) / 1e6, "est_height": -estimate[:, 2] + offset[2],
        "target_s": (target["timestamp"].astype(np.int64) - START_US) / 1e6,
        "target_height": -target["z"] + offset[2], "hold_slice": hover,
        "est_z_err": -est_error[:, 2], "track_z_err": z_target - estimate[hover, 2],
    }
    return metrics, series


def check_ulog_sensors(truth, stamps, groups, dataset):
    metrics = {}
    for name, key, bound, previous in (
        ("sensor_accel", "accel_frd", .020, True),
        ("sensor_gyro", "gyro_frd", .0024, True),
        ("sensor_mag", "mag_sent_frd", 1e-6, False),
    ):
        log = dataset(name)
        keep = log["timestamp_sample"] <= stamps[-1]
        adjusted = {key: vals[keep] for key, vals in log.items()}
        times, indices = clock_match(adjusted, "timestamp_sample", stamps)
        reference = vector(truth, key, 3)
        expected = ((reference[indices] + reference[np.maximum(indices - 1, 0)]) / 2
                    if previous else reference[indices])
        max_error = float(abs(values(adjusted, ("x", "y", "z")) - expected)[1:].max())
        require(max_error <= bound, f"{name} values differ from MuJoCo outside the quantization bound.")
        available = [m.time_usec for _, m in groups["tx"]["HIL_SENSOR"]
                     if times[0] <= m.time_usec <= times[-1]
                     and (name != "sensor_mag" or m.fields_updated & MAG_MASK)]
        coverage = len(times) / len(available)
        require(coverage >= .98, f"{name} ULog time coverage below 98%.")
        metrics[name] = {"max_abs_error": max_error, "sample_coverage": coverage,
                         "logged_hz": (len(times) - 1) * 1e6 / (times[-1] - times[0])}
    return metrics


def replay_step(truth, i):
    # Offline replay creates fresh data objects. The live simulator never assigns qpos or qvel.
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    vz = []
    for with_force in (True, False):
        data = mujoco.MjData(model)
        row = truth[i:i + 1]
        data.qpos[:] = np.r_[vector(row, "position_enu", 3)[0], vector(row, "q_enu", 4)[0]]
        data.qvel[:] = vector(row, "joint_velocity", 6)[0]
        if with_force:
            data.ctrl[:] = vector(row, "thrust", 4)[0]
        mujoco.mj_step(model, data)
        vz.append(float(data.qvel[2]))
    next_logged = float(truth["joint_velocity_2"][i + 1])
    require(abs(vz[0] - next_logged) < 1e-8 and vz[0] - vz[1] > .02,
            "Native state did not respond to motor force as expected.")
    return {"sim_s": float(truth["sim_s"][i]), "with_recorded_force_dv_against_zero_m_s": vz[0] - vz[1],
            "replay_vs_next_integration_abs_error_m_s": abs(vz[0] - next_logged)}


def plot(truth, series, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, layout="constrained")
    axes[0].plot(truth["sim_s"], truth["height_m"], label="MuJoCo truth")
    axes[0].plot(series["pose_s"], series["est_height"], "--", label="PX4 EKF")
    axes[0].plot(series["target_s"], series["target_height"], ":", label="PX4 setpoint")
    axes[0].set_ylabel("Height (m)")
    axes[0].legend(ncol=3, loc="lower center")
    for i in range(4):
        axes[1].plot(truth["sim_s"], truth[f"actuator_force_{i}"], label=f"Motor {i}", alpha=.8)
    axes[1].set_ylabel("Native force (N)")
    axes[1].legend(ncol=4, loc="upper right")
    axes[2].plot(series["pose_s"][series["hold_slice"]], series["est_z_err"], label="EKF − truth")
    axes[2].plot(series["pose_s"][series["hold_slice"]], series["track_z_err"], label="EKF − setpoint")
    axes[2].set_ylabel("Hold error (m)")
    axes[2].set_xlabel("Simulation time (s)")
    axes[2].legend(loc="upper right")
    for axis in axes:
        axis.grid(alpha=.25)
    figure.savefig(path, dpi=155)
    plt.close(figure)


def inspect(folder, image=None):
    truth, stamps, groups, counts, simulator, run = load(folder)
    exchange = check_exchange(truth, stamps, groups)
    stats = {"run": run, "simulation": simulator, "wire_counts": counts, "timing": exchange}
    if run["fault"] is None:
        stats["flight"], series = measure_flight(truth, stamps, groups, ULog(str(folder / "px4.ulg")))
        if image:
            plot(truth, series, image)
    elif run["fault"] == "disconnect":
        require(simulator["completion"] == "deliberate_local_disconnect"
                and not truth["armed"].any() and not truth["thrust_0"].any(), "Disconnect scenario failed.")
    else:
        require(simulator["completion"] == "armed_actuator_timeout" and run["mujoco_exit_code"] == 1
                and truth["armed"][-1] and .5 < run["fault_elapsed_s"] < 2
                and groups["tx"]["HIL_SENSOR"][-1][1].time_usec == stamps[-1] + DT_US,
                "Armed timeout advanced physics without a same-tick command.")
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    results = {run.name: inspect(run, args.out / f"{run.name}.png") for run in args.runs}
    (args.out / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
    print("Verified " + ", ".join(results))


if __name__ == "__main__":
    main()
