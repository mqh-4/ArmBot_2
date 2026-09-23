#!/usr/bin/env python3
import unittest

import numpy as np

from desk_cleaning.depth import depth_median


class DepthTests(unittest.TestCase):
    def test_ignores_invalid_depth(self):
        depth = np.zeros((20, 20), dtype=np.uint16)
        depth[8:13, 8:13] = 500
        depth[10, 10] = 9000
        sample = depth_median(depth, (4, 4, 16, 16), 0.001, 0.1, 1.0, 0.5)
        self.assertAlmostEqual(sample[2], 0.5)

    def test_returns_none_without_samples(self):
        depth = np.zeros((10, 10), dtype=np.uint16)
        self.assertIsNone(depth_median(depth, (1, 1, 8, 8), 0.001, 0.1, 1.0))

    def test_full_bbox_recovers_when_center_roi_is_empty(self):
        depth = np.zeros((20, 20), dtype=np.uint16)
        depth[4:16, 4:16] = 500
        depth[8:13, 8:13] = 0
        sample = depth_median(depth, (4, 4, 16, 16), 0.001, 0.1, 1.0, 1.0)
        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample[2], 0.5)


if __name__ == '__main__':
    unittest.main()
