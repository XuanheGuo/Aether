"""Start the unchanged Iris experiment beside a native Viewer, or replay a capture."""

import argparse
import json
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import psutil


def live(args):
    output = args.output.resolve()
    if output.exists():
        raise ValueError("Use a new output directory.")
    command = [sys.executable]
    if sys.platform == "darwin":
        entry = Path(sys.executable).with_name("mjpython")
        launcher = str(entry) if entry.is_file() else shutil.which("mjpython")
        if launcher is None:
            raise RuntimeError("Install the pinned MuJoCo package to get mjpython.")
        command = [launcher]
    visual = command + ["-m", "prototypes.iris_viewer", "watch", str(output),
                        "--trace", str(output / "viewer-frames.jsonl")]
    if args.close_at is not None:
        visual.extend(["--close-at", str(args.close_at)])
    runner = [sys.executable, "-m", "prototypes.iris_closed_loop.run",
              "--px4-src", str(args.px4_src.resolve()), "--output", str(output),
              "--instance", str(args.instance)]
    viewer = subprocess.Popen(visual, start_new_session=True)
    experiment = None
    cpu, rss = [], []
    viewer_proc = psutil.Process(viewer.pid)
    viewer_proc.cpu_percent()
    viewer_exited_before_experiment = False
    try:
        experiment = subprocess.Popen(runner, start_new_session=True)
        while experiment.poll() is None:
            if viewer.poll() is None:
                try:
                    rss.append(viewer_proc.memory_info().rss)
                    cpu.append(viewer_proc.cpu_percent())
                except psutil.NoSuchProcess:
                    pass
            elif not viewer_exited_before_experiment:
                viewer_exited_before_experiment = True
                if viewer.returncode:
                    print(f"Viewer exited with status {viewer.returncode}. Closed loop continues headless.",
                          file=sys.stderr, flush=True)
            time.sleep(.25)
        print(f"Flight process finished with status {experiment.returncode}. Logs in {output}", flush=True)
        if experiment.returncode == 0:
            completed = subprocess.run(
                [sys.executable, "-m", "prototypes.iris_closed_loop.analyze",
                 str(output), "--out", str(output / "analysis")], check=False,
            )
            if completed.returncode:
                print("Round 6 analysis failed. The original logs remain available.", file=sys.stderr)
            if viewer.poll() is None:
                print("Close the Viewer when finished. Press O there for logs and the plot.", flush=True)
            viewer.wait()
        return experiment.returncode
    except KeyboardInterrupt:
        if experiment is not None and experiment.poll() is None:
            # Let the existing runner stop its native PX4 and simulator children.
            experiment.send_signal(signal.SIGINT)
            experiment.wait(timeout=20)
        return 130
    finally:
        if viewer.poll() is None:
            viewer.terminate()
            viewer.wait(timeout=5)
        if output.is_dir():
            stats = {
                "observer_pid": viewer.pid,
                "observer_exit_code": viewer.returncode,
                "viewer_exited_before_experiment": viewer_exited_before_experiment,
                "viewer_peak_rss_mib": max(rss, default=0) / 2**20,
                "viewer_cpu_one_core_pct_median": sorted(cpu)[len(cpu) // 2] if cpu else None,
                "resource_samples": len(rss),
            }
            (output / "viewer-run.json").write_text(json.dumps(stats, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    experiment = sub.add_parser("live", help="Launch the closed-loop experiment and detached Viewer")
    experiment.add_argument("--px4-src", type=Path, required=True)
    experiment.add_argument("--output", type=Path, required=True)
    experiment.add_argument("--instance", type=int, default=7)
    experiment.add_argument("--close-at", type=float, help="Verification only: close Viewer at this sim time")
    for name in ("replay", "watch"):
        command = sub.add_parser(name, help="Open a recorded capture" if name == "replay" else "Observe a live capture")
        command.add_argument("run", type=Path)
        command.add_argument("--start", type=float)
        command.add_argument("--paused", action="store_true")
        command.add_argument("--trace", type=Path)
        command.add_argument("--close-at", type=float, help="Verification only: close Viewer at this sim time")
    args = parser.parse_args()
    if args.mode == "live":
        return live(args)

    from prototypes.iris_viewer.window import show

    show(args.run.resolve(), replay=args.mode == "replay", start=args.start, paused=args.paused,
         trace=args.trace, close_at=args.close_at)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
