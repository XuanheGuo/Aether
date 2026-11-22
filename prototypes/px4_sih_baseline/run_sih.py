"""Drive the stock PX4 SIH Quadrotor X through takeoff, hold, and land."""

import argparse
import json
import platform
import re
import shutil
import subprocess
import time
from os import environ
from pathlib import Path

import pexpect
import psutil

PX4_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
ARMED = 2
DISARMED = 1
HOLD = 4
HOLD_SECONDS = 5.0
ANSI = r"\x1b\[[0-?]*[ -/]*[@-~]"
PROMPT = rf"\r?\n(?:{ANSI})*pxh> "


def run(source, output, instance):
    source, output = source.resolve(), output.resolve()
    build = source / "build" / "px4_sitl_default"
    binary = build / "bin" / "px4"
    config = build / "etc"
    head = subprocess.check_output(["git", "-C", source, "rev-parse", "HEAD"], text=True).strip()
    if head != PX4_COMMIT or not binary.is_file() or not config.is_dir():
        raise RuntimeError("Build PX4 v1.17.0 using the documented px4_sitl target first.")
    occupied = Path(f"/tmp/px4_lock-{instance}")
    if occupied.exists():
        raise RuntimeError(f"PX4 lock exists for instance {instance}; choose a free --instance.")

    output.mkdir(parents=True, exist_ok=False)
    rootfs = output / "rootfs"
    rootfs.mkdir()

    # Supply a clean instance and log up to shutdown. Keep flight parameters at airframe defaults.
    env = {key: value for key, value in environ.items() if not key.startswith("PX4_")}
    env.update(PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim", PX4_PARAM_SDLOG_MODE="2")
    commands = output / "commands.txt"
    started = time.monotonic()
    samples = []
    with (output / "px4-console.txt").open("w") as console, commands.open("w") as transcript:
        child = pexpect.spawn(str(binary), ["-i", str(instance), str(config)],
                              cwd=str(rootfs), env=env, encoding="utf-8",
                              codec_errors="replace", echo=False, timeout=12)
        child.logfile_read = console
        meter = psutil.Process(child.pid)
        meter.cpu_percent()

        def command(module, *args):
            line = " ".join([module, *args])
            child.sendline(line)
            # The PXH line editor redraws the prompt on each character without a newline.
            child.expect(PROMPT)
            reply = re.sub(ANSI, "", child.before)
            # Drop line-editor redraws while keeping the actual command response.
            reply = reply.split("\r\n", maxsplit=1)[-1].replace("\r", "")
            transcript.write(f"$ {line}\n{reply}\n")
            transcript.flush()
            if "Invalid command:" in reply:
                raise RuntimeError(f"PX4 rejected command {line!r}.")
            return reply

        def wait_for(topic, expected, seconds, *, min_after=None):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                reply = command("listener", topic, "-n", "1")
                fields = dict(re.findall(r"(?m)^\s*([a-z_][a-z_0-9]*):\s+(\S+)", reply))
                samples.append((meter.memory_info().rss, meter.cpu_percent()))
                if (all(fields.get(key) == value for key, value in expected.items())
                        and (min_after is None or time.monotonic() >= min_after)):
                    return fields
                time.sleep(0.5)
            raise RuntimeError(f"PX4 did not report {expected} for {topic} in {seconds}s.")

        try:
            child.expect(PROMPT, timeout=35)
            wait_for("vehicle_status", {"pre_flight_checks_pass": "True", "arming_state": str(DISARMED)}, 55)
            pose = wait_for("vehicle_local_position", {
                "xy_valid": "True", "z_valid": "True", "v_xy_valid": "True", "v_z_valid": "True",
            }, 15)
            command("ver", "all")
            command("commander", "status")
            command("logger", "status")
            command("commander", "takeoff")
            hold = wait_for("vehicle_status", {
                "nav_state": str(HOLD), "arming_state": str(ARMED), "failsafe": "False",
            }, 35)
            wait_for("vehicle_status", {
                "nav_state": str(HOLD), "arming_state": str(ARMED), "failsafe": "False",
            }, 12, min_after=time.monotonic() + HOLD_SECONDS)
            command("commander", "land")
            landed = wait_for("vehicle_land_detected", {"landed": "True"}, 40)
            end = wait_for("vehicle_status", {"arming_state": str(DISARMED)}, 15)
            command("logger", "status")
            command("logger", "stop")
            transcript.write("$ shutdown\n")
            child.sendline("shutdown")
            child.expect(pexpect.EOF, timeout=18)
            child.close()
            exit_code = child.exitstatus
            if exit_code != 0:
                raise RuntimeError(f"PX4 reported exit {exit_code} and signal {child.signalstatus}.")
        finally:
            # On failure clean up this local process only. Successful runs use PX4 shutdown.
            if not child.closed:
                child.close(force=True)

    logs = list((rootfs / "log").rglob("*.ulg"))
    if len(logs) != 1 or not logs[0].stat().st_size:
        raise RuntimeError(f"Expected exactly one native ULog, found {logs}.")
    shutil.copyfile(logs[0], output / "flight.ulg")
    cpu = [value for _, value in samples[1:]]
    report = {
        "px4_commit": head, "px4_version": "v1.17.0", "airframe": "10040_sihsim_quadx",
        "instance": instance, "native_shutdown_exit_code": exit_code,
        "preflight_local_position_valid": all(pose[f"{key}_valid"] == "True"
                                             for key in ("xy", "z", "v_xy", "v_z")),
        "hold_observed": hold["nav_state"] == str(HOLD), "hold_request_s": HOLD_SECONDS,
        "landed": landed["landed"] == "True", "disarmed": end["arming_state"] == str(DISARMED),
        "macos": platform.mac_ver()[0], "architecture": platform.machine(),
        "wall_elapsed_s": round(time.monotonic() - started, 3),
        "resource_samples": len(samples),
        "sampled_peak_px4_rss_mib": round(max(rss for rss, _ in samples) / 2**20, 2),
        "sampled_median_px4_cpu_percent_one_core": round(sorted(cpu)[len(cpu) // 2], 2),
        "sampled_peak_px4_cpu_percent_one_core": round(max(cpu), 2),
    }
    (output / "run.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-src", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New run directory")
    parser.add_argument("--instance", type=int, default=3)
    args = parser.parse_args()
    if not 0 <= args.instance <= 9:
        parser.error("Use an instance number between 0 and 9.")
    report = run(args.px4_src, args.output, args.instance)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
