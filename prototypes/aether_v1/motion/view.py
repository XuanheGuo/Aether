"""Show the live integration state using a detached MuJoCo display copy."""

import time

import mujoco
import mujoco.viewer

from prototypes.aether_v1.motion.experiment import DT, build, run


def view(case, auto_close):
    display = build(case, DT)
    rendered = mujoco.MjData(display)
    frames = 0
    with mujoco.viewer.launch_passive(display, rendered, show_left_ui=False, show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.opt.geomgroup[3] = 0
            viewer.cam.lookat[:] = [0., 0., 1.65 if case == "vertical" else 1.95]
            viewer.cam.distance = 5.2 if case == "vertical" else 4.
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -11
        first = time.monotonic()

        def draw(step, model, data, row):
            nonlocal frames
            if step % 10 or not viewer.is_running():
                return
            target = first + 2 * row["time_s"]
            time.sleep(max(0., target - time.monotonic()))
            with viewer.lock():
                mujoco.mj_copyData(rendered, model, data)
            labels = "Aether Vehicle-V1\nNative dynamics\nTime\nCOM height\nVertical speed\nTest force (+Z)\nContact points"
            values = (f"\n{case}\n{row['time_s']:.3f} s\n{row['com_position_m_z']:.3f} m"
                      f"\n{row['com_velocity_m_s_z']:+.3f} m/s\n{row['applied_test_force_N_z']:.2f} N"
                      f"\n{row['contact_points']}")
            viewer.set_texts((None, None, labels, values))
            viewer.sync()
            frames += 1

        rows, metrics = run(case, on_frame=draw)
        metrics["viewer_synced_frames"] = frames
        metrics["viewer_wall_duration_s"] = time.monotonic() - first
        if auto_close:
            time.sleep(.6)
        else:
            while viewer.is_running():
                time.sleep(.08)
    return rows, metrics
