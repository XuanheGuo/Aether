"""Run MuJoCo and stream native samples over the PX4 simulator TCP channel."""

import argparse
import csv
import importlib.metadata
import io
import json
import math
import select
import socket
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from pymavlink.dialects.v20 import common as mav

MODEL = Path(__file__).parent / "vendor/crazyflie2/cf2.xml"
MODEL_REVISION = "71f066ad0be9cd271f7ed58c030243ef157af9f4"
ENU_TO_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
FLU_TO_FRD = np.diag([1, -1, -1])
REFERENCE = (47.397742, 8.545594, 488.0)
EARTH_RADIUS_M = 6_371_000
GRAVITY_M_S2 = 9.80665
DT_US = 4_000
START_US = 1_000_000
RELEASE_S = 3.0
CAPTURE_S = 5.0
FIELDS = sum(getattr(mav, f"HIL_SENSOR_UPDATED_{field}") for field in (
    "XACC", "YACC", "ZACC", "XGYRO", "YGYRO", "ZGYRO", "XMAG", "YMAG", "ZMAG",
    "ABS_PRESSURE", "PRESSURE_ALT", "TEMPERATURE",
))
COLUMNS = (
    "time_usec", "sim_s", "imu_crc_valid", "north_m", "east_m", "down_m",
    "vn_m_s", "ve_m_s", "vd_m_s", "q_w", "q_x", "q_y", "q_z",
    "ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s", "gz_rad_s",
    "mx_gauss", "my_gauss", "mz_gauss", "abs_pressure_hpa",
)


def transform(rotation, linear_enu, vector_flu):
    """Transform world velocity, local vector, and body-to-world attitude."""
    matrix = ENU_TO_NED @ rotation @ FLU_TO_FRD.T
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, matrix.reshape(-1))
    return ENU_TO_NED @ linear_enu, FLU_TO_FRD @ vector_flu, q


def geodetic(displacement_ned):
    north, east, down = displacement_ned
    lat0, lon0, altitude0 = REFERENCE
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return round(lat * 1e7), round(lon * 1e7), round((altitude0 - down) * 1000)


def scene():
    xml = ET.parse(MODEL).getroot()
    # Leave Menagerie meshes, inertials and actuators intact. The fixture is an experiment layer.
    xml.find("compiler").set("meshdir", str(MODEL.parent.resolve() / "assets"))
    option = xml.find("option")
    if option is None:
        option = ET.SubElement(xml, "option")
    option.set("timestep", str(DT_US / 1e6))
    option.set("gravity", f"0 0 {-GRAVITY_M_S2}")
    option.set("magnetic", "0.03 0.21 -0.43")
    root = xml.find("./worldbody/body[@name='cf2']")
    root.set("pos", "0 0 30")
    ET.SubElement(ET.SubElement(xml, "equality"), "weld", name="fixture", body1="cf2",
                  solref="0.012 1")
    sensors = xml.find("sensor")
    ET.SubElement(sensors, "magnetometer", name="interop_mag", site="imu")
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    return model


class Transport:
    def __init__(self, sock, stream):
        self.sock = sock
        self.stream = stream
        self.buffer = io.BytesIO()
        self.encoder = mav.MAVLink(self.buffer, srcSystem=79, srcComponent=191)
        self.decoder = mav.MAVLink(None)
        self.decoder.robust_parsing = True
        self.types_seen = set()
        self.latest = {}

    def record(self, direction, message, encoded, *, host_ns, valid=True):
        item = {"direction": direction, "host_ns": host_ns, "type": message.get_type(),
                "seq": message.get_seq(), "time_usec": getattr(message, "time_usec", None),
                "valid": valid, "bytes_hex": encoded.hex()}
        self.stream.write(json.dumps(item, separators=(",", ":")) + "\n")

    def send(self, message, bad_crc=False):
        self.buffer.seek(0)
        self.buffer.truncate()
        self.encoder.send(message)
        wire = self.buffer.getvalue()
        if bad_crc:
            wire = wire[:-1] + bytes([wire[-1] ^ 0x01])
        now = time.monotonic_ns()
        self.sock.sendall(wire)
        self.record("tx", message, wire, host_ns=now, valid=not bad_crc)

    def read_until(self, deadline, *, ready=None):
        while (remaining := deadline - time.monotonic()) > 0:
            if not select.select([self.sock], [], [], remaining)[0]:
                break
            incoming = self.sock.recv(65536)
            now = time.monotonic_ns()
            if not incoming:
                return False
            for message in self.decoder.parse_buffer(incoming) or []:
                kind = message.get_type()
                self.types_seen.add(kind)
                if kind != "BAD_DATA":
                    self.latest[kind] = message
                self.record("rx", message, bytes(message.get_msgbuf()), host_ns=now,
                            valid=kind != "BAD_DATA")
            if ready is not None and ready():
                break
        return True


def stream_samples(model, transport, output, disconnect):
    data = mujoco.MjData(model)
    root, site = model.body("cf2").id, model.site("imu").id
    local_fixture = model.equality("fixture").id
    origin_enu = np.array([0, 0, 30.0])
    velocity = np.empty(6)
    step = 0
    bad_tick = round(3.48 * 1e6 / DT_US)
    end_tick = round(CAPTURE_S * 1e6 / DT_US)
    release_tick = round(RELEASE_S * 1e6 / DT_US)
    deadline = time.monotonic()
    completion = "host_timeout"

    with (output / "native.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        # The host pace is open-loop. PX4 advances its clock from the primary IMU packets.
        while step < round(35 * 1e6 / DT_US):
            if step == release_tick:
                data.eq_active[local_fixture] = False
                data.qvel[:] = (.40, -.25, 0, .08, -.11, .65)
            mujoco.mj_forward(model, data)
            if (np.any(data.warning.number) or not np.all(np.isfinite(data.sensordata))
                    or np.any(data.ctrl)):
                raise RuntimeError("Invalid MuJoCo state or unexpected actuator feedback.")
            stamp = START_US + round(data.time * 1e6)
            if stamp != START_US + step * DT_US:
                raise RuntimeError("Native simulation timestep drifted.")
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, site, velocity, 0)
            rotation = data.xmat[root].reshape(3, 3)
            accel_flu = data.sensor("body_linacc").data
            vn, accel, q = transform(rotation, velocity[3:], accel_flu)
            gyro = FLU_TO_FRD @ data.sensor("body_gyro").data
            mag = FLU_TO_FRD @ data.sensor("interop_mag").data
            displacement = ENU_TO_NED @ (data.site_xpos[site] - origin_enu)
            lat, lon, alt_mm = geodetic(displacement)
            altitude = alt_mm / 1000
            pressure = 1013.25 * (1 - 2.25577e-5 * altitude)**5.25588
            bad_crc = step == bad_tick and not disconnect
            writer.writerow([stamp, data.time, int(not bad_crc), *displacement, *vn, *q,
                             *accel, *gyro, *mag, pressure])
            try:
                transport.send(transport.encoder.hil_sensor_encode(
                    stamp, *accel, *gyro, *mag, pressure, 0, altitude, 15.0, FIELDS
                ), bad_crc)
                if step % 5 == 0:
                    velocity_cm_s = [round(x * 100) for x in vn]
                    speed = round(np.linalg.norm(vn[:2]) * 100)
                    course = round(math.degrees(math.atan2(vn[1], vn[0])) * 100) % 36000 if speed else 65535
                    transport.send(transport.encoder.hil_gps_encode(
                        stamp, 3, lat, lon, alt_mm, 80, 120, speed, *velocity_cm_s, course, 12
                    ))
                    acceleration_ned = ENU_TO_NED @ (rotation @ accel_flu + model.opt.gravity)
                    accel_mg = [round(x * 1000 / GRAVITY_M_S2) for x in acceleration_ned]
                    transport.send(transport.encoder.hil_state_quaternion_encode(
                        stamp, q.tolist(), *gyro, lat, lon, alt_mm, *velocity_cm_s,
                        0, round(np.linalg.norm(vn) * 100), *accel_mg
                    ))
                if step == end_tick:
                    stream.flush()
                    transport.stream.flush()
                    (output / "capture-ready").touch()
                    if disconnect:
                        completion = "deliberate_simulator_disconnect"
                        break
                mujoco.mj_step(model, data)
                step += 1
                # Reset wall pacing after stalls instead of delivering bursts to uORB.
                deadline = max(deadline + DT_US / 1e6, time.monotonic())
                if not transport.read_until(deadline):
                    completion = "px4_socket_eof"
                    break
            except (BrokenPipeError, ConnectionResetError):
                completion = "px4_socket_closed"
                break
    if step < end_tick:
        raise RuntimeError(f"Connection ended before the required capture window at step {step}.")
    return {"last_sim_step": step, "completion": completion}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--disconnect", action="store_true")
    args = parser.parse_args()
    model = scene()
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", args.port))
        listener.listen(1)
        listener.settimeout(25)
        (args.output / "bridge-ready").touch()
        conn, address = listener.accept()
        with conn, (args.output / "wire.jsonl").open("w") as wire:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(3)
            transport = Transport(conn, wire)
            until = time.monotonic() + 6
            while (time.monotonic() < until and not (
                {"HEARTBEAT", "COMMAND_LONG"} <= transport.types_seen
            )):
                if not transport.read_until(min(until, time.monotonic() + 0.05)):
                    break
            if not {"HEARTBEAT", "COMMAND_LONG"} <= transport.types_seen:
                raise RuntimeError("Missing PX4 heartbeat or message-interval request.")
            capture = stream_samples(model, transport, args.output, args.disconnect)
    summary = {
        **capture, "model": "Menagerie Bitcraze Crazyflie 2", "revision": MODEL_REVISION,
        "mujoco": mujoco.__version__, "numpy": np.__version__, "tcp_port": args.port,
        "pymavlink": importlib.metadata.version("pymavlink"),
        "peer": list(address), "origin_lla": REFERENCE, "step_us": DT_US,
        "clock_start_us": START_US, "fixture_release_s": RELEASE_S,
        "world_frame": "ENU", "body_frame": "FLU", "wire_world_frame": "NED",
        "wire_body_frame": "FRD", "return_controls_applied": False,
        "rx_message_types": sorted(transport.types_seen),
    }
    (args.output / "bridge.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
