"""Verify physical values and timestamps from native samples through wire and PX4 ULog."""

import argparse
import hashlib
import json
from collections import Counter
from itertools import pairwise
from pathlib import Path

import numpy as np
from pymavlink.dialects.v20 import common as mav
from pyulog import ULog

if __package__:
    from .bridge import DT_US, FIELDS, GRAVITY_M_S2, RELEASE_S, START_US, geodetic
else:
    from bridge import DT_US, FIELDS, GRAVITY_M_S2, RELEASE_S, START_US, geodetic


def read_frames(path):
    counts = Counter()
    groups = {"tx": {}, "rx": {}}
    bad = []
    seqs = {"tx": [], "rx": []}
    for line in path.read_text().splitlines():
        event = json.loads(line)
        side = event["direction"]
        packet = mav.MAVLink(None)
        packet.robust_parsing = True
        messages = packet.parse_buffer(bytes.fromhex(event["bytes_hex"]))
        if not messages:
            raise ValueError("MAVLink record is missing frame bytes.")
        counts[f"{side}:{event['type']}"] += 1
        if not event["valid"]:
            if any(msg.get_type() != "BAD_DATA" for msg in messages):
                raise ValueError("A marked invalid frame passed the MAVLink parser.")
            bad.append(event)
            seqs[side].append(event["seq"])
            continue
        if len(messages) != 1:
            raise ValueError("Unexpected number of decoded messages per wire record.")
        msg = messages[0]
        if msg.get_type() != event["type"] or msg.get_seq() != event["seq"]:
            raise ValueError("The decoded wire frame differs from its metadata.")
        seqs[side].append(msg.get_seq())
        groups[side].setdefault(event["type"], []).append((event, msg))
    gaps = {side: int(sum((b - a - 1) % 256 for a, b in pairwise(seq)))
            for side, seq in seqs.items()}
    return groups, dict(counts), bad, gaps


def values(data, fields):
    return np.column_stack([data[field] for field in fields])


def clock_match(topic, key, stamps):
    measured = topic[key].astype(np.int64)
    indices = np.searchsorted(stamps, measured)
    if np.any(indices >= len(stamps)) or not np.array_equal(stamps[indices], measured):
        raise ValueError(f"PX4 {key} is outside the common MuJoCo clock.")
    if np.any(np.diff(measured) <= 0):
        raise ValueError(f"PX4 {key} is not strictly increasing.")
    return measured, indices


def errors_for(data, supported, physical):
    ulog_errors = {}
    coverage = {}
    raw = {}
    for topic, col, timestamp, bound, conversion, fifo in (
        ("sensor_accel", ("ax_m_s2", "ay_m_s2", "az_m_s2"), "timestamp_sample", 0.020, 1.0, True),
        ("sensor_gyro", ("gx_rad_s", "gy_rad_s", "gz_rad_s"), "timestamp_sample", 0.0024, 1.0, True),
        ("sensor_mag", ("mx_gauss", "my_gauss", "mz_gauss"), "timestamp_sample", 1e-6, 1.0, False),
        ("sensor_baro", ("abs_pressure_hpa",), "timestamp_sample", 0.0003, 100.0, False),
        ("vehicle_local_position_groundtruth", ("north_m", "east_m", "down_m"),
         "timestamp", 0.10, 1.0, False),
        ("vehicle_attitude_groundtruth", ("q_w", "q_x", "q_y", "q_z"),
         "timestamp", 0.010, 1.0, False),
    ):
        fields = data.get_dataset(topic).data
        stamps, indices = clock_match(fields, timestamp, supported)
        reference = values(physical, col)
        expected = reference[indices]
        if fifo:
            # v1.17 reports trapezoids of successive accepted primary IMU samples.
            expected = (expected + reference[np.maximum(indices - 1, 0)]) / 2
        if topic == "sensor_baro":
            measured = values(fields, ("pressure",)) / conversion
        elif topic == "vehicle_attitude_groundtruth":
            measured = values(fields, [f"q[{i}]" for i in range(4)])
        else:
            measured = values(fields, ("x", "y", "z"))
        if topic == "vehicle_attitude_groundtruth":
            dots = (np.sum(expected * measured, axis=1)
                    / (np.linalg.norm(expected, axis=1) * np.linalg.norm(measured, axis=1)))
            residual = 2 * np.arccos(np.clip(abs(dots), 0.0, 1.0))
        else:
            residual = abs(expected - measured)
        peak = float(np.max(residual))
        ulog_errors[topic] = {"max_absolute_error": peak, "limit": bound, "pass": peak <= bound}
        if timestamp == "timestamp_sample":
            possible = np.count_nonzero((supported >= stamps[0]) & (supported <= stamps[-1]))
            coverage[topic] = {"matched": len(stamps), "valid_tx_in_log_window": int(possible),
                               "fraction": len(stamps) / possible}
        raw[topic] = (stamps, measured)
    return ulog_errors, coverage, raw


def inspect_exchange(folder):
    bridge = json.loads((folder / "bridge.json").read_text())
    session = json.loads((folder / "session.json").read_text())
    native = np.genfromtxt(folder / "native.csv", delimiter=",", names=True)
    groups, counts, bad, gaps = read_frames(folder / "wire.jsonl")
    tx, rx = groups["tx"], groups["rx"]
    sensor = tx["HIL_SENSOR"]
    supported = np.array([msg.time_usec for _, msg in sensor], dtype=np.int64)
    supported_rows = native[np.searchsorted(native["time_usec"], supported)]
    if not np.array_equal(supported_rows["time_usec"], supported):
        raise ValueError("Some sensor frames have no originating MuJoCo sample.")
    if not np.all(np.isfinite(native.view(np.float64))) or np.any(
        np.diff(native["time_usec"]) != DT_US
    ) or not np.allclose(START_US + native["sim_s"] * 1e6, native["time_usec"], atol=.001, rtol=0):
        raise ValueError("Invalid MuJoCo samples or clock.")
    if len(bad) != 1 or bad[0]["type"] != "HIL_SENSOR" or bad[0]["direction"] != "tx":
        raise ValueError("Missing exactly one deliberately corrupted test packet.")
    bad_stamp = bad[0]["time_usec"]
    span = np.arange(supported[0], supported[-1] + DT_US, DT_US)
    if not np.array_equal(supported, span[span != bad_stamp]):
        raise ValueError("Unexpected gap, duplicate, or backward step in HIL_SENSOR.")
    mask_error = any(msg.fields_updated != FIELDS or msg.id != 0 for _, msg in sensor)
    motion_fields = ("ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s", "gz_rad_s",
                     "mx_gauss", "my_gauss", "mz_gauss", "abs_pressure_hpa")
    wire_values = np.array([[msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro,
                             msg.xmag, msg.ymag, msg.zmag, msg.abs_pressure]
                            for _, msg in sensor])
    wire_error = float(np.max(abs(wire_values - values(supported_rows, motion_fields))))
    geo = {int(row["time_usec"]): row for row in native}
    quant_error = 0.0
    for kind in ("HIL_GPS", "HIL_STATE_QUATERNION"):
        previous = START_US - 20_000
        for _, message in tx[kind]:
            sample = geo[message.time_usec]
            if message.time_usec - previous != 20_000:
                raise ValueError(f"Unexpected {kind} sample interval.")
            previous = message.time_usec
            expected_pos = geodetic([sample["north_m"], sample["east_m"], sample["down_m"]])
            if (message.lat, message.lon, message.alt) != expected_pos:
                raise ValueError(f"{kind} geography differs from the originating state.")
            velocity = [sample[key] for key in ("vn_m_s", "ve_m_s", "vd_m_s")]
            if kind == "HIL_GPS":
                wire_velocity = (message.vn, message.ve, message.vd)
            else:
                wire_velocity = (message.vx, message.vy, message.vz)
                q = [sample[key] for key in ("q_w", "q_x", "q_y", "q_z")]
                if not np.allclose(message.attitude_quaternion, q, atol=1e-7, rtol=0):
                    raise ValueError("Body-to-NED quaternion changed on the wire.")
            quant_error = max(quant_error, float(np.max(abs(np.array(wire_velocity) / 100 - velocity))))
    ulog = ULog(str(folder / "px4.ulg"))
    ulog_errors, coverage, raw = errors_for(ulog, supported, supported_rows)
    gps = ulog.get_dataset("sensor_gps").data
    gps_times, gps_indices = clock_match(gps, "timestamp", supported)
    gps_rows = supported_rows[gps_indices]
    gps_reference = np.array([
        geodetic([row["north_m"], row["east_m"], row["down_m"]]) for row in gps_rows
    ])
    gps_alt_err = float(np.max(abs(gps["altitude_msl_m"] - gps_reference[:, 2] / 1000)))
    gps_deg_err = float(np.max(abs(
        values(gps, ("latitude_deg", "longitude_deg")) - gps_reference[:, :2] / 1e7)))
    bad_rejected = all(bad_stamp not in stamps for stamps, _ in raw.values())
    bad_rejected &= bool(np.any(raw["sensor_accel"][0] >= bad_stamp + DT_US))

    actuator = rx.get("HIL_ACTUATOR_CONTROLS", [])
    tx_times = {msg.time_usec: event["host_ns"] for event, msg in sensor}
    if len(actuator) < 200:
        raise ValueError(f"PX4 returned only {len(actuator)} actuator frames.")
    pairing = [(msg.time_usec, (event["host_ns"] - tx_times[msg.time_usec]) / 1e6)
               for event, msg in actuator if msg.time_usec in tx_times]
    act_stamps, delay_ms = np.array(pairing).T
    latency = {"pairs": len(delay_ms), "p50_ms": float(np.median(delay_ms)),
               "p95_ms": float(np.quantile(delay_ms, .95)), "max_ms": float(np.max(delay_ms))}
    interval_requests = [msg for _, msg in rx.get("COMMAND_LONG", [])
                         if msg.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL
                         and int(msg.param1) == mav.MAVLINK_MSG_ID_HIL_STATE_QUATERNION]
    act_time_range = (actuator[-1][1].time_usec - actuator[0][1].time_usec) / 1e6
    still = (native["sim_s"] > 1) & (native["sim_s"] < RELEASE_S)
    released = native["sim_s"] >= RELEASE_S
    status = ulog.get_dataset("vehicle_status").data
    checks = {
        "separate_processes": session["px4_pid"] != session["mujoco_pid"],
        "px4_heartbeat_and_state_request": bool(rx.get("HEARTBEAT") and interval_requests),
        "native_static_gravity_and_zero_rates": bool(
            abs(np.median(native["az_m_s2"][still]) + GRAVITY_M_S2) < .03
            and np.max(abs(values(native, motion_fields[3:6])[still])) < .02
        ),
        "native_translation_and_rotation_observed": bool(
            np.ptp(native["down_m"][released]) > 1
            and np.max(abs(native["gz_rad_s"][released])) > .3
            and np.ptp(native["q_z"][released]) > .1
        ),
        "native_values_and_units_preserved_on_wire": wire_error < .0001 and quant_error <= .00501,
        "correct_primary_imu_and_field_mask": not mask_error,
        "ulog_values_match_mujoco": all(result["pass"] for result in ulog_errors.values()),
        "gps_values_match_mujoco": gps_alt_err < .08 and gps_deg_err < 3e-7 and len(gps_times) > 100,
        "bad_crc_rejected_recovery_observed": bad_rejected,
        "zero_unexplained_wire_sequence_gaps": gaps == {"tx": 0, "rx": 0},
        "coverage_of_accepted_imu_within_ulog_window": coverage["sensor_accel"]["fraction"] >= .94,
        "return_clock_monotonic_and_paired": bool(
            np.all(np.diff([msg.time_usec for _, msg in actuator]) >= 0)
            and len(delay_ms) >= .95 * len(actuator) and np.min(delay_ms) >= 0
        ),
        "read_only_disarmed_and_zero_controls": (
            not bridge["return_controls_applied"] and np.all(status["arming_state"] == 1)
            and all(not (msg.mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                    and all(value == 0 for value in msg.controls) for _, msg in actuator)
        ),
        "normal_exit_and_no_ulog_dropout": (
            session["px4_exit_status"] == session["mujoco_exit_status"] == 0
            and not ulog.dropouts and bridge["completion"] in ("px4_socket_eof", "px4_socket_closed")
        ),
    }
    info = {
        "checks": checks, "wire_counts_including_bad_crc": counts, "mavlink_sequence_gaps": gaps,
        "corrupted_packet_time_usec": bad_stamp, "native_sim_clock_step_us": DT_US,
        "imu_configured_hz": 1e6 / DT_US, "gps_and_attitude_configured_hz": 1e6 / (5 * DT_US),
        "actuator_rx_effective_sim_hz": (len(actuator) - 1) / act_time_range,
        "sensor_to_actuator_return_latency": latency, "ulog_capture": coverage,
        "ulog_compare": ulog_errors, "gps_altitude_max_error_m": gps_alt_err,
        "gps_latlon_max_error_deg": gps_deg_err,
        "px4_ulog_sha256": hashlib.sha256((folder / "px4.ulg").read_bytes()).hexdigest(),
        "mujoco_host_interval_p95_ms": float(np.quantile(np.diff([e["host_ns"] for e, _ in sensor])
                                                        / 1e6, .95)),
        "processes": session, "bridge": bridge,
    }
    return info, (native, raw, act_stamps, delay_ms)


def plot(output, dataset):
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    native, raw, act_stamps, delay_ms = dataset
    fig, grid = plt.subplots(2, 2, figsize=(11, 6.7), layout="constrained")
    for ax, topic, field, i, unit in (
        (grid[0, 0], "sensor_accel", "az_m_s2", 2, "FRD accel Z (m/s²)"),
        (grid[0, 1], "sensor_gyro", "gz_rad_s", 2, "FRD gyro Z (rad/s)"),
        (grid[1, 0], "vehicle_local_position_groundtruth", "down_m", 2, "NED down position (m)"),
    ):
        stamp, value = raw[topic]
        ax.plot(native["sim_s"], native[field], "-", lw=1.6, label="MuJoCo")
        ax.plot((stamp - START_US) / 1e6, value[:, i], "--", lw=1.2, label="PX4 ULog")
        ax.axvline(RELEASE_S, color="#a46d37", alpha=.7, ls=":")
        ax.set_ylabel(unit)
        ax.legend(loc="best", fontsize=8)
    lag = grid[1, 1]
    lag.plot((act_stamps - START_US) / 1e6, delay_ms, ".", ms=3, alpha=.55)
    lag.set_ylabel("Matched sensor → actuator return (ms)")
    for ax in grid.flat:
        ax.set_xlabel("MuJoCo simulation time (s)")
        ax.grid(alpha=.22)
    fig.suptitle("Aether · MuJoCo → PX4 v1.17 external SITL · disarmed transport")
    fig.savefig(output / "exchange.png", dpi=150)
    plt.close(fig)


def validate(output, plots=True):
    info, traces = inspect_exchange(output / "exchange")
    failure = output / "disconnect"
    if failure.exists():
        first = json.loads((failure / "session.json").read_text())
        first_bridge = json.loads((failure / "bridge.json").read_text())
        info["disconnect_and_restart"] = first
        info["checks"]["deliberate_disconnect_and_clean_restart"] = (
            first_bridge["completion"] == "deliberate_simulator_disconnect"
            and first["mujoco_exit_status"] == 0 and first["px4_pid"] != info["processes"]["px4_pid"]
            and first["tcp_port"] == info["processes"]["tcp_port"]
        )
    info["checks"] = {key: bool(value) for key, value in info["checks"].items()}
    info["pass"] = all(info["checks"].values())
    (output / "summary.json").write_text(json.dumps(info, indent=2, allow_nan=False) + "\n")
    if plots:
        plot(output, traces)
    print(json.dumps({"pass": info["pass"], "checks": info["checks"],
                      "return_latency_ms": info["sensor_to_actuator_return_latency"]}, indent=2))
    return info["pass"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    raise SystemExit(0 if validate(args.output, plots=not args.no_plots) else 1)
