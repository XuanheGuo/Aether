"""Start stock PX4 and MuJoCo as separate local processes and verify the data path."""

import argparse
import fcntl
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

import pexpect

PX4_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
ANSI = r"\x1b\[[0-?]*[ -/]*[@-~]"
PROMPT = rf"\r?\n(?:{ANSI})*pxh> "
TOPICS = (
    "sensor_accel", "sensor_gyro", "sensor_mag", "sensor_baro", "sensor_gps",
    "vehicle_attitude_groundtruth", "vehicle_local_position_groundtruth",
    "vehicle_local_position", "vehicle_status", "actuator_outputs_sim",
)


def wait_for(path, bridge, timeout):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if path.exists():
            return
        if bridge.poll() is not None:
            raise RuntimeError(f"MuJoCo exited with {bridge.returncode} before {path.name}.")
        time.sleep(.05)
    raise RuntimeError(f"Timed out waiting for {path.name}.")


def run_session(binary, config, output, instance, *, disconnect):
    output.mkdir()
    runtime = output / "rootfs"
    shutil.copytree(config, runtime / "etc")
    logging = runtime / "etc/logging"
    logging.mkdir(exist_ok=True)
    (logging / "logger_topics.txt").write_text("".join(f"{topic} 0 0\n" for topic in TOPICS))
    (output / "logger_topics.txt").write_text((logging / "logger_topics.txt").read_text())
    env = {key: val for key, val in os.environ.items() if not key.startswith("PX4_")}
    env.update(PX4_SIM_MODEL="none_iris", PX4_SIMULATOR="mavlink",
               PX4_SIM_HOST_ADDR="127.0.0.1", PX4_PARAM_SDLOG_MODE="2")
    port = 4560 + instance
    args = [sys.executable, "-B", str(Path(__file__).with_name("bridge.py")),
            "--output", str(output), "--port", str(port)]
    if disconnect:
        args.append("--disconnect")
    start = time.monotonic()
    reconnect = None
    with (output / "bridge-console.txt").open("w") as bridge_log, (
        output / "px4-console.txt"
    ).open("w") as log, (output / "commands.txt").open("w") as commands:
        bridge = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=bridge_log,
                                  stderr=subprocess.STDOUT)
        px4 = None
        try:
            wait_for(output / "bridge-ready", bridge, 20)
            px4 = pexpect.spawn(str(binary), ["-i", str(instance), str(config)],
                                cwd=str(runtime), env=env, encoding="utf-8",
                                codec_errors="replace", echo=False, timeout=12)
            px4.logfile_read = log
            px4.expect(PROMPT, timeout=35)
            wait_for(output / "capture-ready", bridge, 18)

            def command(line):
                px4.sendline(line)
                px4.expect(PROMPT)
                reply = re.sub(ANSI, "", px4.before).split("\r\n", maxsplit=1)[-1]
                commands.write(f"$ {line}\n{reply.replace(chr(13), '')}\n")
                commands.flush()
                if "Invalid command" in reply:
                    raise RuntimeError(f"PX4 rejected {line}.")
                return reply

            if disconnect:
                if bridge.wait(timeout=6) != 0:
                    raise RuntimeError("MuJoCo could not finish the intended disconnect.")
                # v1.17 uses a single TCP connection. Check actual reconnection, then retire PX4.
                with socket.socket() as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    probe.bind(("127.0.0.1", port))
                    probe.listen(1)
                    probe.settimeout(1.5)
                    try:
                        conn, _ = probe.accept()
                    except TimeoutError:
                        reconnect = False
                    else:
                        reconnect = True
                        conn.close()
                commands.write("$ host SIGINT after deliberate simulator disconnect\n")
                px4.kill(2)
            else:
                for topic in ("sensor_accel", "sensor_gyro", "vehicle_attitude_groundtruth"):
                    command(f"listener {topic} -n 1")
                command("commander status")
                command("logger status")
                command("logger stop")
                commands.write("$ shutdown\n")
                px4.sendline("shutdown")
            try:
                px4.expect(pexpect.EOF, timeout=10)
            except pexpect.TIMEOUT:
                if not disconnect:
                    raise
            px4.close(force=disconnect)
            px4_exit, px4_signal = px4.exitstatus, px4.signalstatus
            bridge_exit = bridge.wait(timeout=8)
            if bridge_exit != 0 or (not disconnect and px4_exit != 0):
                raise RuntimeError(f"Unexpected exit status: PX4={px4_exit}, MuJoCo={bridge_exit}.")
        finally:
            if px4 is not None and not px4.closed:
                px4.close(force=True)
            if bridge.poll() is None:
                bridge.terminate()
                try:
                    bridge.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    bridge.kill()
                    bridge.wait()
    logs = list((runtime / "log").rglob("*.ulg"))
    if len(logs) == 1:
        shutil.copyfile(logs[0], output / "px4.ulg")
    elif not disconnect:
        raise RuntimeError("Expected one native PX4 ULog from the normal run.")
    report = {
        "px4_pid": px4.pid, "mujoco_pid": bridge.pid, "tcp_port": port, "instance": instance,
        "px4_exit_status": px4_exit, "px4_exit_signal": px4_signal, "mujoco_exit_status": bridge_exit,
        "deliberate_disconnect": disconnect, "same_process_automatic_reconnect": reconnect,
        "wall_duration_s": round(time.monotonic() - start, 3), "ulog_saved": len(logs) == 1,
    }
    (output / "session.json").write_text(json.dumps(report, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-src", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=7)
    parser.add_argument("--normal-only", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.instance <= 9:
        parser.error("Use a simulator instance between 0 and 9.")
    lock = Path(f"/tmp/px4_lock-{args.instance}")
    try:
        with lock.open("rb") as state:
            fcntl.flock(state, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except FileNotFoundError:
        pass
    except BlockingIOError:
        parser.error(f"PX4 instance {args.instance} is already running.")
    source = args.px4_src.resolve()
    revision = subprocess.check_output(["git", "-C", source, "rev-parse", "HEAD"], text=True).strip()
    build = source / "build/px4_sitl_default"
    binary, config = build / "bin/px4", build / "etc"
    if revision != PX4_COMMIT or not binary.is_file() or not config.is_dir():
        parser.error("Build the pinned PX4 v1.17.0 px4_sitl target before starting.")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    if not args.normal_only:
        run_session(binary, config, output / "disconnect", args.instance, disconnect=True)
    run_session(binary, config, output / "exchange", args.instance, disconnect=False)
    from verify import validate

    return 0 if validate(output, plots=not args.no_plots) else 1


if __name__ == "__main__":
    raise SystemExit(main())
