#!/usr/bin/env python3
import math
import unittest

import cv2
import numpy as np

from desk_cleaning.yolo_onnx import (
    YoloOnnxDetector,
    estimate_axis_angle,
    offset_bbox,
    roi_bounds,
)


class YoloHelperTests(unittest.TestCase):
    def test_letterbox_preserves_aspect_ratio(self):
        detector = object.__new__(YoloOnnxDetector)
        detector.input_size = (640, 640)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        padded, scale, pad_x, pad_y = detector._letterbox(image)
        self.assertEqual(padded.shape, (640, 640, 3))
        self.assertAlmostEqual(scale, 1.0)
        self.assertEqual((pad_x, pad_y), (0, 80))

    def test_axis_angle_for_horizontal_object(self):
        image = np.zeros((120, 200, 3), dtype=np.uint8)
        cv2.rectangle(image, (30, 50), (170, 70), (255, 255, 255), -1)
        angle = estimate_axis_angle(image, (20, 30, 180, 90))
        horizontal_error = min(abs(angle), abs(abs(angle) - math.pi))
        self.assertLess(horizontal_error, math.radians(5))

    def test_roi_bounds_are_pixel_bounds(self):
        self.assertEqual(
            roi_bounds((480, 640, 3), {
                'x_min': 0.05, 'y_min': 0.20, 'x_max': 0.95, 'y_max': 0.98,
            }),
            (32, 96, 608, 470),
        )

    def test_offset_bbox_maps_roi_detection_to_source(self):
        self.assertEqual(offset_bbox((10, 20, 50, 60), 32, 96), (42, 116, 82, 156))


if __name__ == '__main__':
    unittest.main()
