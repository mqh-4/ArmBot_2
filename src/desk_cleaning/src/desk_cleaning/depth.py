#!/usr/bin/env python3
"""Depth-image helpers shared by the RGB-D perception node."""
from typing import Optional, Sequence, Tuple

import numpy as np


def depth_median(
    depth_image,
    bbox: Sequence[int],
    depth_scale: float,
    min_depth: float,
    max_depth: float,
    roi_ratio: float = 0.25,
) -> Optional[Tuple[float, float, float]]:
    """Return center pixel and robust depth from the middle of a bounding box."""
    x1, y1, x2, y2 = [int(value) for value in bbox]
    height, width = depth_image.shape[:2]
    x1, x2 = max(0, x1), min(width, x2)
    y1, y2 = max(0, y1), min(height, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    ratio = min(max(float(roi_ratio), 0.05), 1.0)
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    half_w = max(1, int((x2 - x1) * ratio / 2.0))
    half_h = max(1, int((y2 - y1) * ratio / 2.0))
    roi = depth_image[
        max(0, int(center_y) - half_h):min(height, int(center_y) + half_h + 1),
        max(0, int(center_x) - half_w):min(width, int(center_x) + half_w + 1),
    ]
    values = roi.astype(np.float64) * float(depth_scale)
    valid = values[np.isfinite(values) & (values >= min_depth) & (values <= max_depth)]
    if valid.size < 3:
        return None
    return center_x, center_y, float(np.median(valid))
