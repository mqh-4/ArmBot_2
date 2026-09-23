import math
import unittest

from desk_cleaning.simulation import arm_pulses_to_radians, gripper_pulse_to_radians, interpolate, rotate_vector


class SimulationTest(unittest.TestCase):
    def test_arm_mapping_matches_hiwonder_joint_controllers(self):
        angles = arm_pulses_to_radians([500, 500, 500, 500, 500])
        expected = [0.0] * 5
        for actual, target in zip(angles, expected):
            self.assertAlmostEqual(actual, target)

        angles = arm_pulses_to_radians([600, 600, 600, 600, 600])
        positive = math.radians(24.0)
        expected = [positive, positive, -positive, positive, positive]
        for actual, target in zip(angles, expected):
            self.assertAlmostEqual(actual, target)

    def test_gripper_mapping_uses_servo_controller_zero(self):
        self.assertAlmostEqual(gripper_pulse_to_radians(700), 0.0)
        self.assertAlmostEqual(gripper_pulse_to_radians(200), math.radians(-120))

    def test_interpolation_is_clamped(self):
        self.assertEqual(interpolate(10, 20, -1), 10)
        self.assertEqual(interpolate(10, 20, 0.25), 12.5)
        self.assertEqual(interpolate(10, 20, 2), 20)

    def test_quaternion_rotation(self):
        quarter_turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        rotated = rotate_vector((1.0, 0.0, 0.0), quarter_turn)
        self.assertAlmostEqual(rotated[0], 0.0, places=7)
        self.assertAlmostEqual(rotated[1], 1.0, places=7)
        self.assertAlmostEqual(rotated[2], 0.0, places=7)


if __name__ == '__main__':
    unittest.main()
