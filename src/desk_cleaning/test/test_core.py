#!/usr/bin/env python3
import json
import unittest

from desk_cleaning.core import (
    CleanupCommand,
    Detection,
    PinholeProjector,
    Workspace,
    adaptive_gripper_open_pulse,
    base_to_ik_position,
    parse_command,
    select_target,
    sphere_center_depth,
)


ALIASES = {
    'pen': ['笔', '签字笔'],
    'eraser': ['橡皮', '橡皮擦'],
    'phone': ['手机'],
}
DESTINATIONS = {'left_tray': ['左边'], 'right_tray': ['右边']}


class CommandTests(unittest.TestCase):
    def test_plain_cleanup(self):
        command = parse_command('请帮我清理一下桌面', ALIASES, DESTINATIONS)
        self.assertEqual(command.intent, 'cleanup')
        self.assertEqual(command.allowed_classes, [])

    def test_short_cleanup_command(self):
        command = parse_command('清理', ALIASES, DESTINATIONS)
        self.assertEqual(command.intent, 'cleanup')
        self.assertEqual(command.allowed_classes, [])

    def test_short_cleanup_defaults_to_apple_box_when_configured(self):
        command = parse_command(
            '清理', dict(ALIASES, apple=['苹果']),
            {'default_tray': ['盒子'], 'apple_box': ['放入盒子']},
        )
        self.assertEqual(command.destination, 'apple_box')

    def test_start_work_selects_all_fruits_for_apple_box(self):
        aliases = dict(
            ALIASES,
            apple=['苹果'], orange=['橙子'], banana=['香蕉'],
        )
        command = parse_command('开始作业', aliases, {
            'default_tray': ['盒子'], 'apple_box': ['放入盒子'],
        })
        self.assertEqual(command.intent, 'cleanup')
        self.assertEqual(command.allowed_classes, ['apple', 'orange', 'banana'])
        self.assertEqual(command.destination, 'apple_box')

    def test_grasp_command_is_cleanup_command(self):
        aliases = dict(ALIASES, apple=['苹果', 'apple'])
        command = parse_command('开始夹取苹果', aliases)
        self.assertEqual(command.intent, 'cleanup')
        self.assertEqual(command.allowed_classes, ['apple'])

    def test_grasp_command_can_select_apple_box(self):
        aliases = dict(ALIASES, apple=['苹果', 'apple'])
        command = parse_command(
            '开始夹取苹果放入盒子', aliases,
            {'apple_box': ['放入盒子']},
        )
        self.assertEqual(command.allowed_classes, ['apple'])
        self.assertEqual(command.destination, 'apple_box')

    def test_specific_box_alias_wins_over_generic_box_alias(self):
        aliases = dict(ALIASES, apple=['苹果', 'apple'])
        command = parse_command(
            '开始夹取苹果放入盒子', aliases,
            {
                'default_tray': ['盒子'],
                'apple_box': ['放入盒子'],
            },
        )
        self.assertEqual(command.destination, 'apple_box')

    def test_speech_recognition_variant_jia取_is_accepted(self):
        aliases = dict(ALIASES, apple=['苹果', 'apple'])
        command = parse_command('开始加取苹果放入盒子', aliases, {
            'default_tray': ['盒子'], 'apple_box': ['放入盒子'],
        })
        self.assertEqual(command.allowed_classes, ['apple'])
        self.assertEqual(command.destination, 'apple_box')

    def test_filtered_cleanup(self):
        command = parse_command('只清理签字笔和橡皮，放到左边盒子', ALIASES, DESTINATIONS)
        self.assertEqual(command.allowed_classes, ['pen', 'eraser'])
        self.assertEqual(command.destination, 'left_tray')

    def test_exclusion(self):
        command = parse_command('清理桌面，但不要碰手机', ALIASES, DESTINATIONS)
        self.assertEqual(command.excluded_classes, ['phone'])
        self.assertEqual(command.allowed_classes, [])

    def test_stop(self):
        self.assertEqual(parse_command('马上停止清理', ALIASES).intent, 'stop')

    def test_do_not_clean_object_is_not_stop(self):
        command = parse_command('清理桌面但别清理手机', ALIASES)
        self.assertEqual(command.intent, 'cleanup')
        self.assertEqual(command.excluded_classes, ['phone'])

    def test_chinese_count(self):
        command = parse_command('清理两个物品', ALIASES)
        self.assertEqual(command.max_objects, 2)

    def test_longest_alias_wins(self):
        aliases = {'pen': ['笔'], 'pencil': ['铅笔']}
        command = parse_command('清理铅笔', aliases)
        self.assertEqual(command.allowed_classes, ['pencil'])

    def test_json_rejects_extra_capability(self):
        with self.assertRaises(ValueError):
            CleanupCommand.from_json(json.dumps({'intent': 'cleanup', 'servo': 1000}))

    def test_validation_rejects_wrong_field_types(self):
        command = CleanupCommand.from_json(json.dumps({
            'intent': 'cleanup', 'allowed_classes': 'pen',
        }))
        with self.assertRaises(ValueError):
            command.validate(ALIASES, ['default_tray'])


class GeometryTests(unittest.TestCase):
    def test_base_to_ik_position_matches_urdf_tf_axes(self):
        self.assertEqual(
            base_to_ik_position((0.099, 0.190, 0.102)),
            (0.190, -0.099, 0.102),
        )

    def test_sphere_center_depth_adds_apparent_radius(self):
        self.assertAlmostEqual(
            sphere_center_depth(0.636, (245, 317, 298, 373), 546.13, 546.13),
            0.66865,
            places=4,
        )

    def test_adaptive_gripper_opens_more_for_wider_object(self):
        common = dict(
            base_pulse=70,
            reference_width=0.070,
            width_margin=0.018,
            pulse_per_meter=2000,
            min_pulse=40,
            max_pulse=100,
        )
        narrow = adaptive_gripper_open_pulse((0.050, 0.055), **common)
        apple = adaptive_gripper_open_pulse((0.060, 0.062), **common)
        wide = adaptive_gripper_open_pulse((0.090, 0.090), **common)
        self.assertEqual(narrow, 64)
        self.assertEqual(apple, 50)
        self.assertEqual(wide, 40)

    def test_adaptive_gripper_falls_back_and_clamps(self):
        self.assertEqual(adaptive_gripper_open_pulse(
            (), 70, 0.070, 0.018, 2000, 40, 100,
        ), 70)
        self.assertEqual(adaptive_gripper_open_pulse(
            (0.30, 0.30), 70, 0.070, 0.018, 2000, 40, 100,
        ), 40)

    def test_center_pixel_projects_to_axis(self):
        projector = PinholeProjector(600, 600, 320, 240)
        self.assertEqual(projector.project(320, 240, 0.5), (0.0, 0.0, 0.5))

    def test_workspace_and_selection(self):
        workspace = Workspace(0.1, 0.3, -0.2, 0.2, 0.0, 0.1)
        items = [
            Detection('far', 'pen', '笔', 0.90, (0.27, 0.0, 0.02)),
            Detection('near', 'eraser', '橡皮', 0.95, (0.15, 0.0, 0.02)),
            Detection('outside', 'pen', '笔', 0.99, (0.50, 0.0, 0.02)),
        ]
        self.assertEqual(select_target(items, workspace, [], []).object_id, 'near')
        self.assertEqual(select_target(items, workspace, ['pen'], []).object_id, 'far')

    def test_workspace_radial_reach_excludes_unreachable_corners(self):
        workspace = Workspace(
            -0.345, 0.345, -0.345, 0.345, 0.0, 0.18,
            radial_reach=[
                {'z': 0.00, 'max': 0.345},
                {'z': 0.10, 'max': 0.345},
                {'z': 0.15, 'max': 0.320},
            ],
        )
        self.assertTrue(workspace.contains((-0.096, 0.108, 0.012)))
        self.assertTrue(workspace.contains((0.0, 0.345, 0.02)))
        self.assertFalse(workspace.contains((0.30, 0.30, 0.02)))
        self.assertFalse(workspace.contains((0.33, 0.0, 0.16)))

    def test_workspace_radial_profile_interpolates_by_height(self):
        workspace = Workspace(
            -0.4, 0.4, -0.4, 0.4, 0.0, 0.2,
            radial_reach=[
                {'z': 0.00, 'max': 0.34},
                {'z': 0.10, 'max': 0.30},
            ],
        )
        self.assertAlmostEqual(workspace.radial_limits(0.05)[1], 0.32)
        self.assertTrue(workspace.contains((0.31, 0.0, 0.05)))
        self.assertFalse(workspace.contains((0.33, 0.0, 0.05)))


if __name__ == '__main__':
    unittest.main()
