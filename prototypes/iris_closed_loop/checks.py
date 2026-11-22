"""Fast Iris frame, motor and optional upstream parameter checks."""

import os
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from prototypes.iris_closed_loop.analyze import replay_step
from prototypes.iris_closed_loop.simulator import (
    DT_US,
    K_THRUST,
    MAG_NOISE_DENSITY,
    MAG_PERIOD_US,
    MODEL,
    TIME_DOWN,
    TIME_UP,
    motor_step,
)
from prototypes.mujoco_sitl.bridge import ENU_TO_NED, FLU_TO_FRD, transform


class IrisChecks(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))

    def test_initial_pose_and_units(self):
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)
        rotation = data.xmat[self.model.body("iris").id].reshape(3, 3)
        velocity, acceleration, quat = transform(rotation, [0, 0, 0], -self.model.opt.gravity)
        np.testing.assert_allclose(velocity, [0, 0, 0])
        np.testing.assert_allclose(acceleration, [0, 0, -9.80665], atol=1e-10)
        np.testing.assert_allclose(abs(quat), [1, 0, 0, 0], atol=1e-10)
        np.testing.assert_allclose(ENU_TO_NED @ [1, 2, 3], [2, 1, -3])
        np.testing.assert_allclose(FLU_TO_FRD @ [1, 2, 3], [1, -2, -3])
        self.assertAlmostEqual(self.model.body_subtreemass[self.model.body("iris").id], 1.535)
        self.assertEqual(self.model.opt.timestep * 1e6, DT_US)
        for _ in range(250):
            mujoco.mj_step(self.model, data)
        self.assertGreater(data.ncon, 0)
        np.testing.assert_allclose(data.sensor("accel").data, -self.model.opt.gravity, atol=1e-6)

    def test_motor_transfer(self):
        dt = DT_US / 1e6
        requested = [.2, .4, .6, .8]
        speed, thrust = motor_step(np.zeros(4), requested, True, dt)
        np.testing.assert_allclose(speed, np.array([300, 500, 700, 900]) * (1 - np.exp(-dt / TIME_UP)))
        np.testing.assert_allclose(thrust, K_THRUST * speed**2)
        spinning = np.full(4, 800.)
        slowing, _ = motor_step(spinning, np.zeros(4), True, dt)
        np.testing.assert_allclose(slowing, 100 + 700 * np.exp(-dt / TIME_DOWN))
        np.testing.assert_array_equal(motor_step(spinning, requested, False, dt), np.zeros((2, 4)))
        for bad in ([0] * 3, [0, 0, 0, np.nan], [0, 0, 0, 1.2], [0, 0, 0, -.1]):
            with self.assertRaises(ValueError):
                motor_step(speed, bad, True, dt)

    def test_mujoco_motor_indices_and_directions(self):
        for i, expected in enumerate(((-1, -1, -1), (1, 1, -1), (1, -1, 1), (-1, 1, 1))):
            data = mujoco.MjData(self.model)
            data.qpos[:7] = [0, 0, 1, 1, 0, 0, 0]
            data.ctrl[i] = 5
            mujoco.mj_forward(self.model, data)
            np.testing.assert_array_equal(np.sign(data.qfrc_actuator[3:]), expected)
            self.assertAlmostEqual(data.qfrc_actuator[2], 5)
            np.testing.assert_array_equal(data.actuator_force, np.eye(4)[i] * 5)

    def test_offline_native_replay(self):
        from prototypes.iris_closed_loop.simulator import COLUMNS

        data = mujoco.MjData(self.model)
        data.qpos[2] = 1
        data.ctrl[:] = 4
        truth = np.zeros(2, dtype=[(key, float) for key in COLUMNS])
        for key, vector in (("position_enu", data.qpos[:3]), ("q_enu", data.qpos[3:]),
                            ("joint_velocity", data.qvel), ("thrust", data.ctrl)):
            for i, value in enumerate(vector):
                truth[f"{key}_{i}"][0] = value
        mujoco.mj_step(self.model, data)
        truth["joint_velocity_2"][1] = data.qvel[2]
        self.assertGreater(replay_step(truth, 0)["with_recorded_force_dv_against_zero_m_s"], .04)

    @unittest.skipUnless(os.getenv("IRIS_SDF"), "Set IRIS_SDF to the pinned official Iris SDF template.")
    def test_official_sdf_parameter_parity(self):
        source = ET.parse(os.environ["IRIS_SDF"]).find("model")
        mjcf = ET.parse(MODEL).getroot()
        for source_name, body in (("base_link", "iris"), ("/imu_link", "imu_link"),
                                  *[(f"rotor_{i}", f"rotor_{i}") for i in range(4)]):
            link = source.find(f"link[@name='{source_name}']")
            target = mjcf.find(f".//body[@name='{body}']")
            self.assertEqual(float(link.findtext("inertial/mass")), float(target.find("inertial").get("mass")))
            tensor = link.find("inertial/inertia")
            self.assertEqual([float(tensor.findtext(f"i{axis}{axis}")) for axis in "xyz"],
                             np.fromstring(target.find("inertial").get("diaginertia"), sep=" ").tolist())
            if body != "iris":
                self.assertEqual([float(x) for x in link.findtext("pose").split()[:3]],
                                 np.fromstring(target.get("pos"), sep=" ").tolist())
        geometry = source.find("link[@name='base_link']/collision/geometry/box/size")
        np.testing.assert_allclose(np.fromstring(geometry.text, sep=" "),
                                   np.fromstring(mjcf.find(".//geom[@name='base_contact']").get("size"), sep=" ") * 2)
        channels = source.find("plugin[@name='mavlink_interface']/control_channels")
        for plugin in source.findall("plugin[@filename='libgazebo_motor_model.so']"):
            i = int(plugin.findtext("motorNumber"))
            self.assertEqual(plugin.findtext("linkName"), f"rotor_{i}")
            self.assertEqual(float(plugin.findtext("motorConstant")), K_THRUST)
            self.assertEqual(float(plugin.findtext("timeConstantUp")), TIME_UP)
            self.assertEqual(float(plugin.findtext("timeConstantDown")), TIME_DOWN)
            sign = 1 if plugin.findtext("turningDirection") == "cw" else -1
            self.assertEqual(self.model.actuator_gear[i, 5], sign * float(plugin.findtext("momentConstant")))
            self.assertEqual(int(channels[i].findtext("input_index")), i)
            self.assertEqual(float(channels[i].findtext("input_scaling")), 1000)
            self.assertEqual(float(channels[i].findtext("zero_position_armed")), 100)
        mag = source.find("plugin[@name='magnetometer_plugin']")
        self.assertEqual(float(mag.findtext("noiseDensity")), MAG_NOISE_DENSITY)
        self.assertEqual(float(mag.findtext("pubRate")), 1e6 / MAG_PERIOD_US)


if __name__ == "__main__":
    unittest.main()
