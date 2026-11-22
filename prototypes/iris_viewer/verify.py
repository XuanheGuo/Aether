"""Audit the observer against raw native truth and the unchanged Round 6 acceptance."""

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

from prototypes.iris_closed_loop.analyze import inspect
from prototypes.iris_viewer.records import ARMED, POS, QUAT, STAMP, TIME, recorded_truth


def require(condition, text):
    if not condition:
        raise ValueError(text)


def read_trace(trace, capture):
    rows = recorded_truth(capture / "truth.csv")
    events = [json.loads(line) for line in trace.read_text().splitlines()]
    frames = [event for event in events if event["event"] == "frame"]
    require(len(frames) > 1 and events[-1]["event"] == "closed",
            "Viewer did not report frames and a clean window close.")
    stamps = np.asarray([frame["time_usec"] for frame in frames])
    i = np.searchsorted(rows[:, STAMP], stamps)
    require(np.all(i < len(rows)) and np.array_equal(stamps, rows[i, STAMP]),
            "Viewer timestamp does not correspond to a recorded physics tick.")
    require(np.array_equal([frame["qpos"] for frame in frames],
                           np.column_stack((rows[i, POS], rows[i, QUAT])))
            and np.array_equal([frame["sim_s"] for frame in frames], rows[i, TIME])
            and np.array_equal([frame["armed"] for frame in frames], rows[i, ARMED]),
            "Viewer pose, arming state, or time differs from native truth.")
    return rows, frames, events[-1]


def inspect_live(run, result, early=False):
    rows, frames, close = read_trace(run / "viewer-frames.jsonl", run)
    stats = json.loads((run / "viewer-run.json").read_text())
    require(stats["observer_exit_code"] == 0 and stats["viewer_exited_before_experiment"]
            and stats["observer_pid"] not in (result["run"]["px4_pid"], result["run"]["mujoco_pid"]),
            "The observer did not exit independently.")
    shown = np.asarray([frame["time_usec"] for frame in frames])
    require(np.all(np.diff(shown) >= 0), "Live Viewer timeline moved backwards.")
    mode_counts = Counter(frame["mode"] for frame in frames)
    required_modes = {"Takeoff", "Hold"} if early else {"Takeoff", "Hold", "Land"}
    require(required_modes <= set(mode_counts), "Viewer did not display the expected native flight modes.")
    require(any(frame["target_px4_height_m"] is not None for frame in frames if frame["mode"] == "Hold"),
            "Viewer did not receive a native height target.")
    require(rows[-1, STAMP] > shown[-1] and not rows[-1, ARMED],
            "Physics did not continue to disarmed after window closure.")
    if early:
        require(frames[-1]["mode"] == "Hold" and frames[-1]["armed"]
                and rows[-1, TIME] - frames[-1]["sim_s"] > 5,
                "The early-close run did not exercise continuation from airborne Hold.")
    times = {}
    with (run / "wire.jsonl").open() as source:
        for line in source:
            record = json.loads(line)
            if record["direction"] == "tx" and record["type"] == "HIL_SENSOR":
                times[record["time_usec"]] = record["host_ns"]
    frame_times = np.asarray([frame["host_ns"] for frame in frames])
    age = (frame_times - np.asarray([times[stamp] for stamp in shown])) / 1e6
    require(np.all(age >= 0) and np.all(np.diff(frame_times) > 0), "Observer timing is inconsistent.")
    latency = result["timing"]["host_imu_to_actuator_ms"]
    data = {
        "closed_event": close,
        "sampled_native_modes": dict(mode_counts),
        "frames": len(frames),
        "observer_syncs_per_s": (len(frames) - 1) * 1e9 / (frame_times[-1] - frame_times[0]),
        "native_sample_to_observer_sync_ms": np.percentile(age, [50, 95, 100]).tolist(),
        "last_displayed_time_s": frames[-1]["sim_s"],
        "unsupervised_physics_s_after_window_closed": float(rows[-1, TIME] - frames[-1]["sim_s"]),
        "simulator_round_trip_ms": latency,
    }
    return {**stats, **data}


def inspect_replay(run, trace):
    _, frames, close = read_trace(trace, run)
    stamps = [frame["time_usec"] for frame in frames]
    require(any(a > b for a, b in zip(stamps, stamps[1:], strict=False))
            and any(frame["playing"] for frame in frames)
            and any(not frame["playing"] for frame in frames), "Timeline controls were not exercised.")
    speeds = sorted({frame["rate"] for frame in frames})
    cameras = sorted({frame["camera"] for frame in frames})
    require(len(speeds) > 1 and cameras == ["follow", "free"], "Speed or camera toggle was not exercised.")
    return {"frames_exactly_matched": len(frames), "speeds": speeds, "cameras": cameras, "close": close}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for label in ("headless", "live", "early-close", "replay", "replay-trace", "out"):
        parser.add_argument(f"--{label}", type=Path, required=True)
    args = parser.parse_args()
    results = {}
    for key, run in (("headless", args.headless), ("live", args.live), ("early_close", args.early_close)):
        results[key] = inspect(run)
        if key != "headless":
            results[key]["viewer"] = inspect_live(run, results[key], early=key == "early_close")
    results["replay"] = inspect_replay(args.replay, args.replay_trace)
    args.out.write_text(json.dumps(results, indent=2) + "\n")
    print("Round 6 acceptance passed in all three new flights. Native Viewer and replay traces match truth.")


if __name__ == "__main__":
    main()
