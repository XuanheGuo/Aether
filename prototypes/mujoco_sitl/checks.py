"""Independent frame, timing, native-sensor and MAVLink codec regression tests."""

import io
import json
import socket
import unittest

import mujoco
import numpy as np
from bridge import (
    DT_US,
    FIELDS,
    FLU_TO_FRD,
    GRAVITY_M_S2,
    START_US,
    Transport,
    geodetic,
    scene,
    transform,
)
from pymavlink.dialects.v20 import common as mav
from verify import clock_match


class InteropChecks(unittest.TestCase):
    def test_level_east_maps_to_yaw_ninety_and_down_proper_force(self):
        velocity, acceleration, q = transform(np.eye(3), np.zeros(3), (0, 0, GRAVITY_M_S2))
        np.testing.assert_array_equal(velocity, 0)
        np.testing.assert_array_equal(acceleration, (0, 0, -GRAVITY_M_S2))
        np.testing.assert_allclose(abs(q), [np.sqrt(.5), 0, 0, np.sqrt(.5)], atol=1e-14)

    def test_north_heading_translation_and_body_angular_rate(self):
        north_heading = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        ned, frd, q = transform(north_heading, (2, 3, 4), (0.1, 0.2, 0.3))
        np.testing.assert_array_equal(ned, (3, 2, -4))
        np.testing.assert_array_equal(frd, (0.1, -0.2, -0.3))
        np.testing.assert_allclose(abs(q), (1, 0, 0, 0), atol=1e-14)
        reference = geodetic((0, 0, 0))
        moved = geodetic((3, 2, -4))
        self.assertGreater(moved[0], reference[0])
        self.assertGreater(moved[1], reference[1])
        self.assertEqual(moved[2] - reference[2], 4000)

    def test_native_fixture_then_unpowered_motion(self):
        model = scene()
        data = mujoco.MjData(model)
        for _ in range(150):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(FLU_TO_FRD @ data.sensor("body_linacc").data,
                                   (0, 0, -GRAVITY_M_S2), atol=.03)
        data.eq_active[model.equality("fixture").id] = False
        data.qvel[:] = (.2, -.3, 0, .1, -.2, .3)
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.sensor("body_gyro").data, (.1, -.2, .3), atol=1e-10)
        previous = data.qpos[:3].copy()
        for _ in range(20):
            mujoco.mj_step(model, data)
        self.assertLess(data.qpos[2], previous[2])
        self.assertGreater(data.qpos[0], previous[0])
        np.testing.assert_array_equal(data.ctrl, 0)

    def test_clock_accepts_shared_ticks_and_rejects_oob_duplicate_or_reverse(self):
        clock = np.arange(START_US, START_US + 4 * DT_US, DT_US)
        np.testing.assert_array_equal(clock_match({"sample": clock[::2]}, "sample", clock)[1], (0, 2))
        for unhealthy in ([clock[-1] + DT_US], [clock[0], clock[0]], [clock[2], clock[1]]):
            with self.subTest(unhealthy=unhealthy), self.assertRaises(ValueError):
                clock_match({"sample": np.array(unhealthy)}, "sample", clock)

    def test_pymavlink_fragmentation_crc_and_recovery(self):
        tx, rx = socket.socketpair()
        with tx, rx:
            log = io.StringIO()
            link = Transport(tx, log)
            for index in range(3):
                message = link.encoder.hil_sensor_encode(
                    START_US + index * DT_US, 0, 0, -GRAVITY_M_S2,
                    .1, -.2, .3, .03, .21, -.43, 1013, 0, 0, 15, FIELDS
                )
                link.send(message, bad_crc=index == 1)
            entries = [json.loads(line) for line in log.getvalue().splitlines()]
            expected = b"".join(bytes.fromhex(entry["bytes_hex"]) for entry in entries)
            payload = rx.recv(len(expected), socket.MSG_WAITALL)
            self.assertEqual(payload, expected)
            codec = mav.MAVLink(None)
            codec.robust_parsing = True
            decoded = []
            for start in range(0, len(payload), 7):
                decoded.extend(codec.parse_buffer(payload[start:start + 7]) or [])
            good = [msg for msg in decoded if msg.get_type() != "BAD_DATA"]
            self.assertEqual([msg.time_usec for msg in good], [START_US, START_US + 2 * DT_US])
            self.assertTrue(any(msg.get_type() == "BAD_DATA" for msg in decoded))
            self.assertEqual([msg.get_seq() for msg in good], [0, 2])
            self.assertTrue(all(msg.fields_updated == FIELDS for msg in good))


if __name__ == "__main__":
    unittest.main()
