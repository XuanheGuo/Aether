"""Read native Iris states and advance MuJoCo with same-tick PX4 motor commands."""

import argparse
import csv
import json
import math
import socket
import time
from pathlib import Path

import mujoco
import numpy as np
from pymavlink.dialects.v20 import common as mav

from prototypes.mujoco_sitl.bridge import (
    DT_US,
    ENU_TO_NED,
    FIELDS,
    FLU_TO_FRD,
    GRAVITY_M_S2,
    REFERENCE,
    START_US,
    Transport,
    geodetic,
    transform,
)

MODEL = Path(__file__).with_name("iris.xml")
UPSTREAM_REVISION = "6697ab169ceab512dc706acea63df4c882662c60"
K_THRUST = 5.84e-6
TIME_UP = .0125
TIME_DOWN = .025
MAG_PERIOD_US = 10_000
MAG_NOISE_DENSITY = .0004
MAG_MASK = (mav.HIL_SENSOR_UPDATED_XMAG | mav.HIL_SENSOR_UPDATED_YMAG
            | mav.HIL_SENSOR_UPDATED_ZMAG)
MAG_SEED = 26
ACTUATOR = "HIL_ACTUATOR_CONTROLS"
VECTORS = {"position_enu": 3, "q_enu": 4, "joint_velocity": 6,
           "velocity_ned": 3, "q_ned": 4, "accel_frd": 3, "gyro_frd": 3,
           "mag_native_frd": 3, "mag_sent_frd": 3, "control": 4, "rotor_speed": 4,
           "thrust": 4, "actuator_force": 4}
COLUMNS = [
    "time_usec", "sim_s", "armed", "lockstep_flag", "matched", "return_time_usec",
    "return_latency_ms", "height_m", "contacts",
    *[f"{key}_{i}" for key, count in VECTORS.items() for i in range(count)],
]


def motor_step(speed, output, armed, dt):
    """Apply the published Iris input scaling, speed lag and squared thrust law."""
    command = np.asarray(output, dtype=float)
    if command.shape != (4,) or not np.all(np.isfinite(command)):
        raise ValueError("Incomplete or nonfinite four-motor vector.")
    if np.any(command < -1e-6) or np.any(command > 1 + 1e-6):
        raise ValueError(f"PX4 reported a non-reversible motor outside [0, 1]: {command}")
    if not armed:
        return np.zeros(4), np.zeros(4)
    target = 100 + 1000 * np.clip(command, 0, 1)
    tau = np.where(target >= speed, TIME_UP, TIME_DOWN)
    rotor_speed = target + (speed - target) * np.exp(-dt / tau)
    return rotor_speed, K_THRUST * rotor_speed**2


class TimedTransport(Transport):
    def __init__(self, sock, stream):
        super().__init__(sock, stream)
        self.sent_ns = None
        self.received_ns = None

    def record(self, direction, message, encoded, *, host_ns, valid=True):
        kind = message.get_type()
        if direction == "tx" and kind == "HIL_SENSOR":
            self.sent_ns = host_ns
        if direction == "rx" and kind == ACTUATOR:
            self.received_ns = host_ns
        super().record(direction, message, encoded, host_ns=host_ns, valid=valid)


def drive(model, link, root, fault_at):
    data = mujoco.MjData(model)
    body = model.body("iris").id
    site = model.site("truth").id
    mujoco.mj_forward(model, data)
    origin = data.site_xpos[site].copy()
    velocity = np.empty(6)
    speed = np.zeros(4)
    step = 0
    match_count = bootstrap_misses = armed_ticks = 0
    saw_controller = False
    last_armed = False
    termination = "maximum_duration"
    next_wall_step = time.monotonic()
    rng = np.random.default_rng(MAG_SEED)
    last_mag_update = -MAG_PERIOD_US
    mag = np.zeros(3)
    with (root / "truth.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        while step < 60_000:
            mujoco.mj_forward(model, data)
            moved = data.site_xpos[site] - origin
            stamp = START_US + round(data.time * 1e6)
            if stamp != START_US + step * DT_US:
                raise RuntimeError("MuJoCo timestamp does not equal the requested step.")
            if np.any(data.warning.number) or not np.all(np.isfinite(data.sensordata)):
                raise RuntimeError("Nonfinite measurement or MuJoCo warning.")
            rotation = data.xmat[body].reshape(3, 3)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, site, velocity, 0)
            accel_flu = data.sensor("accel").data
            vn, accel, q_ned = transform(rotation, velocity[3:], accel_flu)
            gyro = FLU_TO_FRD @ data.sensor("gyro").data
            native_mag = FLU_TO_FRD @ data.sensor("mag").data
            mag_update = stamp // MAG_PERIOD_US > last_mag_update // MAG_PERIOD_US
            if mag_update:
                interval_s = min(stamp - last_mag_update, 2 * MAG_PERIOD_US) / 1e6
                mag = native_mag + rng.normal(0, MAG_NOISE_DENSITY / math.sqrt(interval_s), 3)
                last_mag_update = stamp
            fields = FIELDS if mag_update else FIELDS & ~MAG_MASK
            lat, lon, alt_mm = geodetic(ENU_TO_NED @ moved)
            altitude = alt_mm / 1000
            pressure = 1013.25 * (1 - 2.25577e-5 * altitude)**5.25588
            try:
                link.send(link.encoder.hil_sensor_encode(
                    stamp, *accel, *gyro, *mag, pressure, 0, altitude, 15.0, fields
                ))
                if step % 5 == 0:
                    cm_s = [round(x * 100) for x in vn]
                    s2d = round(np.linalg.norm(vn[:2]) * 100)
                    cog = round(math.degrees(math.atan2(vn[1], vn[0])) * 100) % 36000 if s2d else 65535
                    link.send(link.encoder.hil_gps_encode(
                        stamp, 3, lat, lon, alt_mm, 80, 120, s2d, *cm_s, cog, 12
                    ))
                    acceleration_ned = ENU_TO_NED @ (rotation @ accel_flu + model.opt.gravity)
                    milli_g = [round(x * 1000 / GRAVITY_M_S2) for x in acceleration_ned]
                    link.send(link.encoder.hil_state_quaternion_encode(
                        stamp, q_ned.tolist(), *gyro, lat, lon, alt_mm, *cm_s,
                        0, round(np.linalg.norm(vn) * 100), *milli_g
                    ))
                if fault_at is not None and data.time >= fault_at - 1e-10:
                    termination = "deliberate_local_disconnect"
                    break

                def ready(expected_stamp=stamp):
                    message = link.latest.get(ACTUATOR)
                    return message is not None and message.time_usec >= expected_stamp

                timeout_s = .6 if last_armed else (.15 if saw_controller else .012)
                if not link.read_until(time.monotonic() + timeout_s, ready=ready):
                    termination = "armed_socket_disconnect" if last_armed else "px4_socket_eof"
                    break
            except (BrokenPipeError, ConnectionResetError):
                termination = "armed_socket_disconnect" if last_armed else "px4_socket_closed"
                break

            feedback = link.latest.get(ACTUATOR)
            matched = feedback is not None and feedback.time_usec == stamp
            if feedback is not None and feedback.time_usec > stamp:
                raise RuntimeError("PX4 returned an actuator message from a future tick.")
            if not matched and last_armed:
                termination = "armed_actuator_timeout"
                break
            if matched:
                saw_controller = True
                match_count += 1
                flag = bool(feedback.flags & 1)
                if not flag:
                    raise RuntimeError("PX4 lockstep bit is absent.")
                armed = bool(feedback.mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                command = feedback.controls[:4]
                return_stamp = feedback.time_usec
                latency = (link.received_ns - link.sent_ns) / 1e6
            else:
                bootstrap_misses += 1
                if step > 375 and not saw_controller:
                    raise RuntimeError("PX4 never returned an actuator frame.")
                armed, flag, command, return_stamp, latency = False, False, [0] * 4, -1, -1

            speed, thrust = motor_step(speed, command, armed, model.opt.timestep)
            data.ctrl[:] = thrust
            mujoco.mj_forward(model, data)
            writer.writerow([
                stamp, data.time, int(armed), int(flag), int(matched), return_stamp, latency,
                moved[2], data.ncon, *data.qpos[:3], *data.qpos[3:7], *data.qvel,
                *vn, *q_ned, *accel, *gyro, *native_mag, *mag, *command, *speed, *thrust, *data.actuator_force,
            ])
            if step % 100 == 0:
                stream.flush()
                link.stream.flush()
            last_armed = armed
            armed_ticks += int(armed)
            mujoco.mj_step(model, data)
            step += 1
            next_wall_step = max(next_wall_step + model.opt.timestep, time.monotonic())
            time.sleep(max(0, next_wall_step - time.monotonic()))
    return {
        "completion": termination, "steps": step, "same_tick_returns": match_count,
        "no_return_while_disarmed": bootstrap_misses, "armed_ticks": armed_ticks,
        "total_mass_kg": float(model.body_subtreemass[body]),
        "nominal_timestep_us": DT_US, "origin_lla": REFERENCE, "iris_sdf_revision": UPSTREAM_REVISION,
        "mujoco_version": mujoco.__version__, "mag_white_noise_density_gauss_sqrt_hz": MAG_NOISE_DENSITY,
        "mag_seed": MAG_SEED, "mag_nominal_hz": 100,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--disconnect-at", type=float)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", args.port))
        listener.listen(1)
        listener.settimeout(30)
        (args.output / "ready").touch()
        conn, _ = listener.accept()
        with conn, (args.output / "wire.jsonl").open("w") as stream:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(3)
            link = TimedTransport(conn, stream)
            until = time.monotonic() + 6
            while time.monotonic() < until and "HEARTBEAT" not in link.types_seen:
                if not link.read_until(min(until, time.monotonic() + .05)):
                    break
            if "HEARTBEAT" not in link.types_seen:
                raise RuntimeError("PX4 did not send its simulator handshake heartbeat.")
            report = drive(model, link, args.output, args.disconnect_at)
    (args.output / "simulator.json").write_text(json.dumps(report, indent=2) + "\n")
    return int(report["completion"] in {"armed_actuator_timeout", "armed_socket_disconnect", "maximum_duration"})


if __name__ == "__main__":
    raise SystemExit(main())
