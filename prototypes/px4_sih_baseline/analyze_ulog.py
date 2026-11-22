"""Validate stock PX4 SIH flight phases and plot native ULog tracking errors."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

ARMED, DISARMED = 2, 1
TAKEOFF, HOLD, LAND = 17, 4, 18


def hold(sample_times, stream_times, values):
    index = np.searchsorted(stream_times, sample_times, side="right") - 1
    return values[np.clip(index, 0, len(stream_times) - 1)]


def interpolate(sample_times, stream_times, values):
    return np.column_stack([np.interp(sample_times, stream_times, axis) for axis in values.T])


def vectors(data, names):
    return np.column_stack([data[name] for name in names])


def quaternion_distance(lhs, rhs):
    with np.errstate(divide="ignore", invalid="ignore"):
        cosine = (np.sum(lhs * rhs, axis=1)
                  / (np.linalg.norm(lhs, axis=1) * np.linalg.norm(rhs, axis=1)))
    return 2 * np.rad2deg(np.arccos(np.clip(abs(cosine), 0, 1)))


def heading(q):
    w, x, y, z = q.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))


def rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def load_data(log_path):
    log = ULog(str(log_path))
    datasets = {item.name: item.data for item in log.data_list if item.multi_id == 0}
    required = {
        "vehicle_status", "vehicle_land_detected", "vehicle_local_position",
        "vehicle_local_position_setpoint", "vehicle_local_position_groundtruth",
        "vehicle_attitude", "vehicle_attitude_setpoint", "vehicle_attitude_groundtruth",
        "actuator_motors",
    }
    if missing := required - set(datasets):
        raise ValueError(f"ULog lacks expected stock SITL topics: {sorted(missing)}")
    return log, datasets


def evaluate(log_path):
    log, datasets = load_data(log_path)

    def clock(data):
        return (data["timestamp"].astype(np.int64) - log.start_timestamp) * 1e-6

    status = datasets["vehicle_status"]
    flight = status["arming_state"] == ARMED
    if not np.any(flight):
        raise ValueError("The ULog has no armed samples.")
    start_index, end_index = np.flatnonzero(flight)[[0, -1]]
    status_time = clock(status)
    begin, end = status_time[start_index], status_time[end_index]
    nav = status["nav_state"]
    armed_nav = nav[flight]
    modes = [int(mode) for i, mode in enumerate(armed_nav)
             if i == 0 or mode != armed_nav[i - 1]]
    desired_modes = [TAKEOFF, HOLD, LAND]

    position = datasets["vehicle_local_position"]
    time = clock(position)
    position_xyz = vectors(position, ("x", "y", "z")).astype(float)
    velocity = vectors(position, ("vx", "vy", "vz"))
    setpoint = datasets["vehicle_local_position_setpoint"]
    commanded = hold(time, clock(setpoint), vectors(setpoint, ("x", "y", "z")))
    truth_position = datasets["vehicle_local_position_groundtruth"]
    truth = interpolate(time, clock(truth_position), vectors(truth_position, ("x", "y", "z")))
    land = datasets["vehicle_land_detected"]
    on_ground = hold(time, clock(land), land["landed"]).astype(bool)
    position_modes = hold(time, status_time, nav)
    armed = hold(time, status_time, status["arming_state"]) == ARMED
    hovering = armed & (position_modes == HOLD) & ~on_ground
    airborne = armed & ~on_ground
    if not np.any(hovering):
        raise ValueError("The ULog has no armed, airborne hold samples.")
    baseline = (time >= begin - 1) & (time < begin)
    if not np.any(baseline):
        raise ValueError("No prearm local position samples.")
    reference_est = np.median(position_xyz[baseline], axis=0)
    reference_truth = np.median(truth[baseline], axis=0)
    state_error = (position_xyz - reference_est) - (truth - reference_truth)
    tracking = position_xyz - commanded
    height = -(position_xyz[:, 2] - reference_est[2])
    height_true = -(truth[:, 2] - reference_truth[2])
    height_goal = -(commanded[:, 2] - reference_est[2])

    attitude = datasets["vehicle_attitude"]
    att_time = clock(attitude)
    q = vectors(attitude, [f"q[{i}]" for i in range(4)]).astype(float)
    desired = datasets["vehicle_attitude_setpoint"]
    attitude_control_error = quaternion_distance(
        q, hold(att_time, clock(desired), vectors(desired, [f"q_d[{i}]" for i in range(4)]))
    )
    truth_attitude = datasets["vehicle_attitude_groundtruth"]
    q_truth = hold(att_time, clock(truth_attitude),
                   vectors(truth_attitude, [f"q[{i}]" for i in range(4)]))
    attitude_estimator_error = quaternion_distance(q, q_truth)
    heading_error = np.rad2deg((heading(q) - heading(q_truth) + np.pi) % (2 * np.pi) - np.pi)
    hover_on_attitude_clock = hold(att_time, time, hovering).astype(bool)
    motors = datasets["actuator_motors"]
    motor_time = clock(motors)
    outputs = vectors(motors, [f"control[{i}]" for i in range(4)])
    hover_on_motor_clock = hold(motor_time, time, hovering).astype(bool)
    valid = np.logical_and.reduce([position[field].astype(bool)
                                   for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")])
    self_checks = {
        "stock_sih_quadrotor_airframe": log.initial_parameters.get("SYS_AUTOSTART") == 10040,
        "ordered_flight_modes": modes == desired_modes,
        "estimator_valid_while_airborne": bool(np.all(valid[airborne])),
        "no_failsafe_while_armed": not bool(np.any(status["failsafe"][flight])),
        "detector_reports_landing_after_takeoff": bool(
            np.any((clock(land) > time[hovering][-1]) & land["landed"].astype(bool))
        ),
        "automatic_disarm_in_log": bool(np.any(status["arming_state"][end_index + 1:] == DISARMED)),
        "ulog_has_no_dropout": not log.dropouts,
        "four_finite_positive_motors_during_hold": bool(
            np.any(hover_on_motor_clock) and np.all(outputs[hover_on_motor_clock] > 0)
            and np.all(np.isfinite(outputs[hover_on_motor_clock]))
        ),
    }
    metrics = {
        "peak_estimated_height_m": float(np.max(height[airborne])),
        "hold_duration_s": float(time[hovering][-1] - time[hovering][0]),
        "hold_altitude_tracking_rmse_m": rms(tracking[hovering, 2]),
        "hold_horizontal_tracking_rmse_m": rms(np.linalg.norm(tracking[hovering, :2], axis=1)),
        "hold_attitude_tracking_rmse_deg": rms(attitude_control_error[hover_on_attitude_clock]),
        "hold_altitude_estimator_rmse_m": rms(state_error[hovering, 2]),
        "hold_attitude_estimator_rmse_deg": rms(attitude_estimator_error[hover_on_attitude_clock]),
        "full_log_peak_attitude_estimator_error_deg": float(np.nanmax(attitude_estimator_error)),
        "full_log_final_heading_estimator_error_deg": float(heading_error[-1]),
    }
    max_bounds = {
        "hold_altitude_tracking_rmse_m": 0.4, "hold_horizontal_tracking_rmse_m": 0.7,
        "hold_attitude_tracking_rmse_deg": 5.0, "hold_altitude_estimator_rmse_m": 0.5,
        "hold_attitude_estimator_rmse_deg": 5.0,
    }
    min_bounds = {"peak_estimated_height_m": 1.5, "hold_duration_s": 4.0}
    passed = (all(self_checks.values()) and all(metrics[k] <= v for k, v in max_bounds.items())
              and all(metrics[k] >= v for k, v in min_bounds.items()))
    report = {
        "pass": passed, "ulg_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
        "ulg_duration_s": round((log.last_timestamp - log.start_timestamp) * 1e-6, 6),
        "px4_git_hash": log.msg_info_dict.get("ver_sw"),
        "aircraft": log.initial_parameters.get("SYS_AUTOSTART"),
        "mode_sequence_armed": modes, "arm_time_s": begin, "last_armed_sample_s": end,
        "checks": self_checks, "metrics": metrics, "max_thresholds": max_bounds,
        "min_thresholds": min_bounds,
    }
    traces = {
        "time": time, "altitude": height, "altitude_truth": height_true, "altitude_setpoint": height_goal,
        "position": position_xyz, "velocity": velocity, "position_mode": position_modes,
        "tracking": tracking, "estimation": state_error, "att_time": att_time,
        "att_control": attitude_control_error, "att_estimation": attitude_estimator_error,
        "motor_time": motor_time, "motors": outputs, "hold_interval": time[hovering][[0, -1]],
    }
    return report, traces


def write_csv(path, traces):
    time = traces["time"]
    table = np.column_stack([
        time, traces["position"], traces["velocity"], traces["position_mode"],
        traces["altitude"], traces["altitude_truth"], traces["altitude_setpoint"],
        -traces["tracking"][:, 2], -traces["estimation"][:, 2],
        np.interp(time, traces["att_time"], traces["att_control"], left=np.nan, right=np.nan),
        np.interp(time, traces["att_time"], traces["att_estimation"], left=np.nan, right=np.nan),
    ])
    header = ("time_s", "north_m", "east_m", "down_m", "vn_m_s", "ve_m_s", "vd_m_s", "nav_state",
              "altitude_est_m", "altitude_sih_m", "altitude_setpoint_m",
              "altitude_control_error_m", "altitude_estimation_error_m",
              "attitude_control_error_deg", "attitude_estimation_error_deg")
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(table)


def plot(path, traces):
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout="constrained")
    t = traces["time"]
    altitude, position_error, attitude_error, motors = axes.ravel()
    altitude.plot(t, traces["altitude"], label="EKF estimate")
    altitude.plot(t, traces["altitude_truth"], "--", label="SIH truth")
    altitude.plot(t, traces["altitude_setpoint"], ":", label="PX4 setpoint")
    altitude.set_ylabel("Height relative to prearm (m)")
    position_error.plot(t, -traces["tracking"][:, 2], label="Estimate − setpoint")
    position_error.plot(t, -traces["estimation"][:, 2], label="Estimate − SIH truth")
    position_error.set_ylabel("Vertical error (m)")
    attitude_error.plot(traces["att_time"], traces["att_control"], label="Estimate vs setpoint")
    attitude_error.plot(traces["att_time"], traces["att_estimation"], label="Estimate vs SIH truth")
    attitude_error.set_ylabel("Attitude error (deg)")
    for i in range(4):
        motors.plot(traces["motor_time"], traces["motors"][:, i], label=f"Motor {i + 1}")
    motors.set_ylabel("Actuator command (normalized)")
    for ax in axes.ravel():
        ax.axvspan(*traces["hold_interval"], color="#91c9c4", alpha=0.23)
        ax.set_xlabel("Time since ULog start (s)")
        ax.grid(alpha=0.2)
        ax.legend(loc="best", fontsize=8)
    fig.suptitle("PX4 v1.17.0 · Stock SIH Quadrotor X\nTakeoff, hold and landing")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New analysis directory")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    report, traces = evaluate(args.ulog)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    write_csv(args.output / "flight.csv", traces)
    if not args.no_plot:
        plot(args.output / "flight.png", traces)
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
