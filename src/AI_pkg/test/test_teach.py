"""Unit tests for the pure drag-teach helpers (no ROS needed)."""
import unittest

from ai_pkg.motion_teach import (MAX_POSITION, clamp_position, command_plan,
                                 max_delta, motion_bounds, plan_duration,
                                 plan_summary, resample, servo_ids, smooth,
                                 trim_idle)


def take(rows):
    """rows: list of (t, {id: pos})"""
    return [(float(t), dict(positions)) for t, positions in rows]


class SampleHelpersTests(unittest.TestCase):
    def test_clamp_position_rounds_and_bounds(self):
        self.assertEqual(clamp_position(-5), 0)
        self.assertEqual(clamp_position(1500), MAX_POSITION)
        self.assertEqual(clamp_position(499.6), 500)

    def test_servo_ids_union(self):
        self.assertEqual(servo_ids(take([(0, {1: 5}), (1, {2: 7})])), [1, 2])

    def test_max_delta(self):
        self.assertEqual(max_delta({1: 500, 2: 600}, {1: 500, 2: 590}), 10)
        # A servo missing from one sample is treated as unchanged, not as 0.
        self.assertEqual(max_delta({1: 500}, {2: 600}), 0)

    def test_motion_bounds_ignores_static_take(self):
        self.assertIsNone(motion_bounds(take([(0, {3: 500}), (1, {3: 501})])))

    def test_motion_bounds_finds_first_and_last_move(self):
        samples = take([
            (0.0, {3: 500}), (0.1, {3: 505}), (0.2, {3: 530}),
            (0.3, {3: 560}), (0.4, {3: 560}), (0.5, {3: 560}),
        ])
        self.assertEqual(motion_bounds(samples, tick_threshold=3), (1, 3))


class TrimTests(unittest.TestCase):
    def test_trim_removes_lead_and_tail(self):
        samples = take(
            # 0.0-0.9 s still, 1.0-1.9 s moving, 2.0-3.4 s still again
            [(round(index * 0.1, 3), {3: 500}) for index in range(10)]
            + [(round(1.0 + index * 0.1, 3), {3: 520 + index * 15}) for index in range(10)]
            + [(round(2.0 + index * 0.1, 3), {3: 655}) for index in range(15)]
        )
        trimmed = trim_idle(samples, tick_threshold=3, pad_seconds=0.3)
        self.assertAlmostEqual(trimmed[0][0], 0.7, places=6)
        self.assertAlmostEqual(trimmed[-1][0], 2.2, places=6)

    def test_trim_keeps_short_take_pairs(self):
        samples = take([(0.0, {3: 500}), (0.1, {3: 590})])
        trimmed = trim_idle(samples)
        self.assertEqual(len(trimmed), 2)
        self.assertEqual(trimmed[-1][1][3], 590)


class SmoothTests(unittest.TestCase):
    def test_smooth_with_alpha_one_is_passthrough(self):
        samples = take([(0.0, {3: 500}), (0.1, {3: 520}), (0.2, {3: 540})])
        result = smooth(samples, alpha=1.0)
        for (_, before), (_, after) in zip(samples, result):
            self.assertAlmostEqual(before[3], after[3])

    def test_smooth_reduces_step(self):
        samples = take([(0.0, {3: 500}), (0.1, {3: 600})])
        result = smooth(samples, alpha=0.25)
        self.assertAlmostEqual(result[1][1][3], 525.0)

    def test_smooth_rejects_bad_alpha(self):
        with self.assertRaises(ValueError):
            smooth(take([(0.0, {3: 500})]), alpha=0)


class ResampleTests(unittest.TestCase):
    def test_uniform_spacing_and_endpoints(self):
        samples = take([(0.0, {3: 500}), (1.0, {3: 600})])
        grid = resample(samples, 10.0)
        self.assertEqual(grid[0][0], 0.0)
        self.assertAlmostEqual(grid[-1][0], 1.0)
        gaps = {round(b[0] - a[0], 6) for a, b in zip(grid, grid[1:])}
        self.assertEqual(len(gaps), 1)
        self.assertAlmostEqual(list(gaps)[0], 0.1, places=6)

    def test_linear_interpolation_midpoint(self):
        samples = take([(0.0, {3: 500}), (1.0, {3: 600})])
        grid = resample(samples, 10.0)
        middle = [positions[3] for time, positions in grid if abs(time - 0.5) < 1e-9]
        self.assertTrue(middle)
        self.assertAlmostEqual(middle[0], 550.0)

    def test_rejects_bad_rate(self):
        with self.assertRaises(ValueError):
            resample(take([(0.0, {3: 500})]), 0)


class CommandPlanTests(unittest.TestCase):
    def setUp(self):
        self.samples = take([
            (0.0, {3: 500, 6: 500}),
            (0.1, {3: 520, 6: 500}),
            (0.2, {3: 540, 6: 505}),
            (0.3, {3: 540, 6: 505}),
        ])

    def test_first_entry_moves_every_servo_with_entry_duration(self):
        plan = command_plan(self.samples, entry_duration=1.5)
        offset, entries = plan[0]
        self.assertEqual(offset, 0.0)
        self.assertEqual([entry[0] for entry in entries], [3, 6])
        self.assertTrue(all(entry[2] == 1.5 for entry in entries))

    def test_only_changed_servos_are_commanded(self):
        plan = command_plan(self.samples, min_delta=2)
        later_ids = [servo_id for _, entries in plan[1:] for servo_id, _, _ in entries]
        self.assertIn(3, later_ids)
        self.assertIn(6, later_ids)          # 5 ticks > min_delta
        self.assertNotIn(6, [i for _, e in plan[1:2] for i, _, _ in e])

    def test_static_take_produces_single_message(self):
        plan = command_plan(take([(0.0, {3: 500}), (0.2, {3: 500})]))
        self.assertEqual(len(plan), 1)

    def test_min_delta_suppresses_tiny_noise(self):
        plan = command_plan(take([(0.0, {3: 500}), (0.1, {3: 501})]), min_delta=2)
        self.assertEqual(len(plan), 1)

    def test_durations_are_bounded(self):
        plan = command_plan(self.samples, min_delta=1)
        for _, entries in plan[1:]:
            for _, _, duration in entries:
                self.assertGreaterEqual(duration, 0.02)
                self.assertLessEqual(duration, 0.5)

    def test_duration_and_summary(self):
        plan = command_plan(self.samples)
        # The last sample repeats the previous pose, so the last command lands
        # at 0.2 s; the arm then simply holds that pose.
        self.assertAlmostEqual(plan_duration(plan), 0.2)
        summary = plan_summary(plan)
        self.assertEqual(summary['duration'], 0.2)
        self.assertEqual(summary['messages'], len(plan))
        self.assertEqual(summary['commands'], sum(len(e) for _, e in plan))

    def test_validation_errors(self):
        with self.assertRaises(ValueError):
            command_plan([])
        with self.assertRaises(ValueError):
            command_plan(self.samples, entry_duration=0)
        with self.assertRaises(ValueError):
            command_plan(self.samples, min_delta=0)
        with self.assertRaises(ValueError):
            command_plan(take([(0.0, {})]))


if __name__ == '__main__':
    unittest.main()
