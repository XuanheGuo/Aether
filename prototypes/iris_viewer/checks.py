"""Isolated checks of render geometry, exact seeking and live CSV buffering."""

import hashlib
import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np

from prototypes.iris_closed_loop.simulator import MODEL
from prototypes.iris_viewer.records import FIELDS, POS, STAMP, TIME, Timeline, TruthTail, recorded_truth
from prototypes.iris_viewer.scene import display_model


class ViewerChecks(unittest.TestCase):
    def test_display_model_is_a_clone_with_non_colliding_visuals(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        visual = display_model()
        self.assertEqual((visual.nq, visual.nv, visual.nu), (model.nq, model.nv, model.nu))
        self.assertEqual(model.opt.timestep, visual.opt.timestep)
        for name in ("body_pos", "body_quat", "body_mass", "body_inertia", "actuator_gear", "qpos0"):
            np.testing.assert_array_equal(getattr(model, name), getattr(visual, name))
        visuals = [visual.geom("fuselage_visual"), *(visual.geom(f"prop_{i}_visual") for i in range(4))]
        self.assertTrue(all(g.contype.item() == g.conaffinity.item() == 0 for g in visuals))
        self.assertEqual(visual.nmesh, 3)

    def test_tail_holds_partial_rows_and_replays_exact_values(self):
        with tempfile.TemporaryDirectory() as folder:
            capture = Path(folder) / "truth.csv"
            capture.write_text(",".join(FIELDS) + "\r\n" + ",".join(["1"] * len(FIELDS)))
            tail = TruthTail(capture)
            self.assertEqual(len(tail.read()), 0)
            with capture.open("a") as stream:
                stream.write("\n" + ",".join(["2"] * len(FIELDS)) + "\n")
            streamed = tail.read()
            before = hashlib.sha256(capture.read_bytes()).digest()
            np.testing.assert_array_equal(streamed, recorded_truth(capture))
            self.assertEqual(hashlib.sha256(capture.read_bytes()).digest(), before)
            self.assertEqual(len(tail.read()), 0)

    def test_play_pause_seek_speed_and_ends(self):
        samples = np.zeros((11, len(FIELDS)))
        samples[:, TIME] = np.arange(11)
        samples[:, STAMP] = 1_000_000 + samples[:, TIME] * 1e6
        samples[:, POS] = np.arange(11)[:, None] / 10
        timeline = Timeline(samples, 2.)
        np.testing.assert_array_equal(timeline.update(4.1), samples[2])
        timeline.key(32, 4.1)
        np.testing.assert_array_equal(timeline.update(10.), samples[2])
        timeline.key(262, 10.)
        np.testing.assert_array_equal(timeline.update(10.), samples[3])
        timeline.key(ord("9"), 10.)
        np.testing.assert_array_equal(timeline.update(10.), samples[-1])
        timeline.key(ord("R"), 10.)
        timeline.key(ord("]"), 10.)
        timeline.key(32, 10.)
        np.testing.assert_array_equal(timeline.update(11.), samples[2])
        np.testing.assert_array_equal(timeline.update(40.), samples[-1])
        self.assertFalse(timeline.playing)
        timeline.key(32, 40.)
        np.testing.assert_array_equal(timeline.update(40.), samples[0])


if __name__ == "__main__":
    unittest.main()
