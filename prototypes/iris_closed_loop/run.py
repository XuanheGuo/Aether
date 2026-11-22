"""Run the official PX4 Iris takeoff, hold and land commands on the MuJoCo port."""

import argparse
import fcntl
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pexpect
import psutil

from prototypes.mujoco_sitl.run import ANSI, PROMPT, PX4_COMMIT, wait_for

TOPICS = (
    "sensor_accel", "sensor_gyro", "sensor_gps", "sensor_baro", "sensor_mag",
    "vehicle_attitude_groundtruth", "vehicle_local_position_groundtruth",
    "vehicle_local_position", "vehicle_status", "actuator_outputs_sim", "actuator_motors",
    "vehicle_land_detected", "vehicle_attitude", "vehicle_attitude_setpoint",
    "vehicle_local_position_setpoint", "estimator_status_flags", "failsafe_flags",
)


def run(source, output, instance, fault):
    source, output = source.resolve(), output.resolve()
    build = source / "build/px4_sitl_default"
    binary, etc = build / "bin/px4", build / "etc"
    revision = subprocess.check_output(["git", "-C", source, "rev-parse", "HEAD"], text=True).strip()
    if revision != PX4_COMMIT or not binary.is_file():
        raise ValueError("Use the pinned native PX4 v1.17.0 px4_sitl build.")
    try:
        with Path(f"/tmp/px4_lock-{instance}").open("rb") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except FileNotFoundError:
        pass
    except BlockingIOError as exc:
        raise RuntimeError(f"Instance {instance} is active.") from exc

    output.mkdir(parents=True, exist_ok=False)
    runtime = output / "rootfs"
    shutil.copytree(etc, runtime / "etc")
    config = runtime / "etc/logging"
    config.mkdir(exist_ok=True)
    topics = "".join(f"{topic} 0 0\n" for topic in TOPICS)
    (config / "logger_topics.txt").write_text(topics)
    (output / "logger_topics.txt").write_text(topics)
    env = {key: val for key, val in os.environ.items() if not key.startswith("PX4_")}
    env.update(PX4_SIM_MODEL="gazebo-classic_iris", PX4_SIMULATOR="mavlink",
               PX4_SIM_HOST_ADDR="127.0.0.1", PX4_PARAM_SDLOG_MODE="2")
    port = 4560 + instance
    args = [sys.executable, "-B", "-m", "prototypes.iris_closed_loop.simulator",
            "--port", str(port), "--output", str(output)]
    if fault == "disconnect":
        args += ["--disconnect-at", "2"]
    started = time.monotonic()
    resource_samples = []
    stage = "startup"
    report = {"arm_and_fly_requested": fault != "disconnect", "fault": fault, "px4_commit": revision,
              "port": port, "instance": instance, "automatic_reconnect_observed": None}
    with (output / "px4-console.txt").open("w") as console, (
        output / "simulator-console.txt"
    ).open("w") as sim_log, (output / "commands.txt").open("w") as transcript:
        sim = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=sim_log, stderr=subprocess.STDOUT)
        px4 = None
        try:
            wait_for(output / "ready", sim, 20)
            px4 = pexpect.spawn(str(binary), ["-i", str(instance), str(etc.resolve())],
                                cwd=str(runtime), env=env, encoding="utf-8",
                                codec_errors="replace", echo=False, timeout=12)
            px4.logfile_read = console
            report.update(px4_pid=px4.pid, mujoco_pid=sim.pid)
            processes = {"px4": psutil.Process(px4.pid), "mujoco": psutil.Process(sim.pid)}
            for meter in processes.values():
                meter.cpu_percent()
            px4.expect(PROMPT, timeout=35)

            def command(line):
                if sim.poll() is not None:
                    raise RuntimeError(f"MuJoCo stopped while attempting {line}.")
                px4.sendline(line)
                px4.expect(PROMPT)
                answer = re.sub(ANSI, "", px4.before).split("\r\n", maxsplit=1)[-1]
                transcript.write(f"$ {line}\n{answer.replace(chr(13), '')}\n")
                transcript.flush()
                if "Invalid command" in answer:
                    raise RuntimeError(f"PX4 rejected {line}.")
                return answer

            def wait_topic(topic, expected, wait, min_after=None):
                deadline = time.monotonic() + wait
                while time.monotonic() < deadline:
                    answer = command(f"listener {topic} -n 1")
                    fields = dict(re.findall(r"(?m)^\s*([a-z_][a-z_0-9]*):\s+(\S+)", answer))
                    resource_samples.append({name: (p.memory_info().rss, p.cpu_percent())
                                             for name, p in processes.items()})
                    if (all(fields.get(key) == val for key, val in expected.items())
                            and (min_after is None or time.monotonic() >= min_after)):
                        return
                    time.sleep(.25)
                raise RuntimeError(f"PX4 did not report {expected} for {topic}.")

            if fault == "disconnect":
                stage = "disconnect"
                if sim.wait(timeout=12) != 0:
                    raise RuntimeError("Simulator failed before deliberate socket disconnect.")
                with socket.socket() as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    probe.bind(("127.0.0.1", port))
                    probe.listen(1)
                    probe.settimeout(1.5)
                    try:
                        conn, _ = probe.accept()
                    except TimeoutError:
                        report["automatic_reconnect_observed"] = False
                    else:
                        report["automatic_reconnect_observed"] = True
                        conn.close()
                transcript.write("$ host SIGINT after deliberate simulator disconnect\n")
                px4.kill(signal.SIGINT)
            else:
                stage = "preflight"
                try:
                    wait_topic("vehicle_status", {"pre_flight_checks_pass": "True", "arming_state": "1"}, 28)
                except RuntimeError:
                    for line in ("commander check", "listener failsafe_flags -n 1",
                                 "listener estimator_status_flags -n 1", "ekf2 status",
                                 "listener vehicle_local_position -n 1"):
                        command(line)
                    raise
                wait_topic("vehicle_local_position", {
                    "xy_valid": "True", "z_valid": "True", "v_xy_valid": "True", "v_z_valid": "True",
                }, 15)
                command("commander status")
                stage = "takeoff"
                command("commander takeoff")
                wait_topic("vehicle_status", {"nav_state": "4", "arming_state": "2", "failsafe": "False"}, 35)
                if fault == "timeout":
                    stage = "armed_timeout"
                    transcript.write("$ host SIGSTOP on the spawned PX4 process\n")
                    fault_started = time.monotonic()
                    px4.kill(signal.SIGSTOP)
                    if sim.wait(timeout=5) != 1:
                        raise RuntimeError("Simulator did not stop on armed actuator timeout.")
                    report["fault_elapsed_s"] = round(time.monotonic() - fault_started, 3)
                    if json.loads((output / "simulator.json").read_text())["completion"] != "armed_actuator_timeout":
                        raise RuntimeError("Simulator exited for a different reason.")
                    px4.kill(signal.SIGCONT)
                    transcript.write("$ host SIGCONT then SIGINT after the simulator abort\n")
                    px4.kill(signal.SIGINT)
                else:
                    stage = "hold"
                    wait_topic("vehicle_status", {"nav_state": "4", "arming_state": "2", "failsafe": "False"},
                               15, min_after=time.monotonic() + 5)
                    stage = "landing"
                    command("commander land")
                    wait_topic("vehicle_land_detected", {"landed": "True"}, 35)
                    wait_topic("vehicle_status", {"arming_state": "1"}, 15)
                    command("logger status")
                    command("logger stop")
                    stage = "shutdown"
                    transcript.write("$ shutdown\n")
                    px4.sendline("shutdown")
            px4.expect(pexpect.EOF, timeout=12)
            px4.close()
            report["px4_exit_code"] = px4.exitstatus
            report["mujoco_exit_code"] = sim.wait(timeout=8)
            if px4.exitstatus != 0 or sim.returncode != int(fault == "timeout"):
                raise RuntimeError(f"Unexpected exit statuses PX4={px4.exitstatus}, MuJoCo={sim.returncode}.")
            stage = "completed"
        finally:
            if px4 is not None and not px4.closed:
                if fault == "timeout":
                    px4.kill(signal.SIGCONT)
                px4.kill(signal.SIGINT)
                try:
                    px4.expect(pexpect.EOF, timeout=5)
                except pexpect.TIMEOUT:
                    pass
                px4.close(force=True)
            if sim.poll() is None:
                sim.terminate()
                try:
                    sim.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    sim.kill()
                    sim.wait()
            logs = sorted((runtime / "log").rglob("*.ulg"))
            if logs:
                shutil.copyfile(logs[-1], output / "px4.ulg")
            report.update(stage=stage, wall_duration_s=round(time.monotonic() - started, 3))
            if resource_samples:
                report["resources"] = {
                    name: {
                        "samples": len(resource_samples),
                        "peak_rss_mib": round(max(s[name][0] for s in resource_samples) / 2**20, 2),
                        "median_cpu_percent_one_core": sorted(s[name][1] for s in resource_samples)[
                            len(resource_samples) // 2],
                    } for name in processes
                }
            (output / "run.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-src", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--instance", type=int, default=7)
    fault = parser.add_mutually_exclusive_group()
    fault.add_argument("--disconnect-test", dest="fault", action="store_const", const="disconnect")
    fault.add_argument("--timeout-test", dest="fault", action="store_const", const="timeout")
    args = parser.parse_args()
    if not 0 <= args.instance <= 9:
        parser.error("Instance must be between 0 and 9.")
    print(json.dumps(run(args.px4_src, args.output, args.instance, args.fault), indent=2))


if __name__ == "__main__":
    main()
