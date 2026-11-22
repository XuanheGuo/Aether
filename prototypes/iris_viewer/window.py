"""A detached MuJoCo passive Viewer. All displayed poses come from truth CSV."""

import json
import queue
import subprocess
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

from prototypes.iris_viewer.records import (
    ARMED, FIELDS, HEIGHT, POS, QUAT, STAMP, TIME, VEL, FlightLog, Timeline, TruthTail,
    latest_ulog, load_active_ulog, recorded_truth,
)
from prototypes.iris_viewer.scene import COLLISION_GROUP, attitude, decorate, display_model

PERIOD = 1 / 30
BAR_SIZE = 28


def summary(viewer, sample, flight_log, caption, timeline, wall, following, completed):
    title = "Recorded Replay" if timeline else "Live PX4 x MuJoCo"
    if sample is None:
        viewer.set_texts((None, None, title, "Waiting for native truth samples"))
        return None, None
    stamp = int(sample[STAMP])
    mode, target = flight_log.at(stamp)
    e, n, u = sample[POS]
    vn, ve, vd = sample[VEL]
    roll, pitch, yaw = attitude(sample[QUAT])
    target_text = "Awaiting setpoint" if target is None else f"{target:.2f} m (PX4 datum)"
    armed = "Armed" if sample[ARMED] else "Disarmed"
    help_text = "Mouse orbit / zoom / pan  |  F: free / follow  |  Q: close"
    action = "O: logs and error plot when available"
    if timeline:
        cursor = timeline.position(wall)
        fraction = (cursor - timeline.start) / max(1e-6, timeline.end - timeline.start)
        fill = round(BAR_SIZE * fraction)
        action = (f"[{'=' * fill}{'-' * (BAR_SIZE - fill)}]  {cursor:.2f} / {timeline.end:.2f} s"
                  f"  |  {timeline.speed:g}x  {'Playing' if timeline.playing else 'Paused'}"
                  "\nSpace: play / pause   R: reset   Left / Right: seek 1 s"
                  "\n0-9: seek across recording   [ / ]: speed   O: logs")
    elif completed:
        title += "  |  End of capture"
    columns = (
        f"{title}\n{caption}\nTime / mode\nArm state\nHeight / target\nPosition ENU"
        "\nVelocity NED\nRoll / pitch / yaw\nCamera / path",
        f"\n\n{sample[TIME]:.3f} s  /  {mode}\n{armed}\n{sample[HEIGHT]:.2f} m  /  {target_text}"
        f"\n{e:+.2f}  {n:+.2f}  {u:+.2f} m\n{vn:+.2f}  {ve:+.2f}  {vd:+.2f} m/s"
        f"\n{roll:+.1f}  {pitch:+.1f}  {yaw:+.1f} deg (ENU/FLU)"
        f"\n{'Follow Iris' if following else 'Free'}  /  recorded ENU trace",
    )
    viewer.set_texts([
        (None, mujoco.mjtGridPos.mjGRID_TOPLEFT, *columns),
        (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
         help_text + "\n" + action, None),
    ])
    return mode, target


def open_capture(run):
    command = "open" if sys.platform == "darwin" else "xdg-open"
    filename = f"{run.name}.png"
    locations = (run / "analysis" / filename, run.parent.parent / "analysis" / filename)
    plot = next((path for path in locations if path.is_file()), None)
    for path in (run, plot):
        if path is not None:
            subprocess.Popen([command, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def show(run, *, replay, start=None, paused=False, trace=None, close_at=None):
    model = display_model()
    rendered = mujoco.MjData(model)
    mujoco.mj_forward(model, rendered)
    if replay:
        recorded = recorded_truth(run / "truth.csv")
        timeline = Timeline(recorded, time.monotonic())
        if start is not None:
            timeline.seek(start, time.monotonic())
        timeline.playing = not paused
        path_samples = recorded[::25]
        flight_log = FlightLog(latest_ulog(run))
    else:
        timeline = None
        flight_log = FlightLog()
        tail = TruthTail(run / "truth.csv")
        path_samples = np.empty((0, len(FIELDS)))
    messages = queue.SimpleQueue()
    native_keys = queue.SimpleQueue()
    done = threading.Event()
    if not replay:
        threading.Thread(target=load_active_ulog, args=(run, done, messages.put), daemon=True).start()
    following = False
    sample = None
    log = None
    reason = "window_closed"
    num_frames = 0
    body = model.body("iris").id
    try:
        with mujoco.viewer.launch_passive(
            model, rendered, key_callback=native_keys.put, show_left_ui=False, show_right_ui=False,
        ) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = [0., 0., 1.15]
                viewer.cam.distance = 4.25
                viewer.cam.azimuth = 132
                viewer.cam.elevation = -21
                viewer.opt.geomgroup[COLLISION_GROUP] = 0
            while viewer.is_running():
                wall = time.monotonic()
                while not native_keys.empty():
                    key = native_keys.get_nowait()
                    if key == ord("Q"):
                        viewer.close()
                        break
                    if key == ord("F"):
                        following = not following
                    elif key == ord("O") and run.is_dir():
                        open_capture(run)
                    elif timeline:
                        timeline.key(key, wall)
                if not viewer.is_running():
                    break
                while not messages.empty():
                    flight_log = messages.get_nowait()

                if timeline:
                    sample = timeline.update(wall)
                elif tail.path.is_file():
                    added = tail.read()
                    if len(added):
                        sample = added[-1]
                        path_samples = np.concatenate((path_samples, added[added[:, STAMP] % 100_000 == 0]))
                if trace is not None and log is None and trace.parent.is_dir():
                    log = trace.open("x", buffering=1)
                with viewer.lock():
                    desired = (mujoco.mjtCamera.mjCAMERA_TRACKING if following
                               else mujoco.mjtCamera.mjCAMERA_FREE)
                    if viewer.cam.type != desired:
                        viewer.cam.type = desired
                        if following:
                            viewer.cam.trackbodyid = body
                    if sample is not None:
                        rendered.time = sample[TIME]
                        rendered.qpos[:3] = sample[POS]
                        rendered.qpos[3:7] = sample[QUAT]
                        mujoco.mj_forward(model, rendered)
                        selected = path_samples[path_samples[:, TIME] <= sample[TIME], POS][-420:]
                    else:
                        selected = ()
                    decorate(viewer.user_scn, selected)
                mode, target = summary(viewer, sample, flight_log, run.name, timeline, wall,
                                       following, (run / "run.json").is_file())
                viewer.sync()
                num_frames += 1
                if sample is not None and log is not None:
                    log.write(json.dumps({
                        "event": "frame", "host_ns": time.monotonic_ns(), "time_usec": int(sample[STAMP]),
                        "sim_s": float(sample[TIME]), "armed": bool(sample[ARMED]),
                        "qpos": rendered.qpos.tolist(), "mode": mode, "target_px4_height_m": target,
                        "camera": "follow" if following else "free",
                        "playing": timeline.playing if timeline else None,
                        "rate": timeline.speed if timeline else None,
                    }) + "\n")
                if close_at is not None and sample is not None and sample[TIME] >= close_at:
                    reason = "test_close"
                    viewer.close()
                    break
                time.sleep(max(0., PERIOD - (time.monotonic() - wall)))
    finally:
        done.set()
        if log is not None:
            log.write(json.dumps({"event": "closed", "host_ns": time.monotonic_ns(),
                                  "reason": reason, "frames": num_frames}) + "\n")
            log.close()
