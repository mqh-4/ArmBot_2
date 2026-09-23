#!/usr/bin/env python3
"""Small OpenCV-DNN wrapper for YOLOv8-style ONNX detectors."""
from dataclasses import dataclass
from typing import List, Mapping, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class BoxDetection:
    class_id: int
    confidence: float
    bbox: Tuple[int, int, int, int]


def roi_bounds(image_shape, roi: Mapping[str, float]):
    """Return clamped pixel bounds for a normalized x/y ROI.

    Bounds use the usual half-open image convention: ``x2`` and ``y2`` are
    suitable for NumPy slicing. Invalid or empty ROIs raise ValueError so a
    bad parameter cannot silently produce an empty detector input.
    """
    image_h, image_w = image_shape[:2]
    x_min = min(max(float(roi.get('x_min', 0.0)), 0.0), 1.0)
    y_min = min(max(float(roi.get('y_min', 0.0)), 0.0), 1.0)
    x_max = min(max(float(roi.get('x_max', 1.0)), 0.0), 1.0)
    y_max = min(max(float(roi.get('y_max', 1.0)), 0.0), 1.0)
    if x_max <= x_min or y_max <= y_min:
        raise ValueError('ROI has no area: %r' % (roi,))
    x1, y1 = int(round(x_min * image_w)), int(round(y_min * image_h))
    x2, y2 = int(round(x_max * image_w)), int(round(y_max * image_h))
    x1, y1 = max(0, min(x1, image_w - 1)), max(0, min(y1, image_h - 1))
    x2, y2 = max(x1 + 1, min(x2, image_w)), max(y1 + 1, min(y2, image_h))
    return x1, y1, x2, y2


def offset_bbox(bbox, x_offset, y_offset):
    """Map an ROI-local ``(x1, y1, x2, y2)`` box back to the source image."""
    x1, y1, x2, y2 = bbox
    return (int(x1 + x_offset), int(y1 + y_offset),
            int(x2 + x_offset), int(y2 + y_offset))


class YoloOnnxDetector:
    def __init__(
        self,
        model_path: str,
        labels: Sequence[str],
        input_size=(640, 640),
        confidence_threshold=0.55,
        nms_threshold=0.45,
    ):
        if not model_path:
            raise ValueError('model_path is empty')
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.labels = list(labels)
        self.input_size = tuple(input_size)
        self.confidence_threshold = float(confidence_threshold)
        self.nms_threshold = float(nms_threshold)

    def _letterbox(self, image):
        height, width = image.shape[:2]
        target_w, target_h = self.input_size
        scale = min(target_w / float(width), target_h / float(height))
        resized_w, resized_h = int(round(width * scale)), int(round(height * scale))
        resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        pad_x = (target_w - resized_w) // 2
        pad_y = (target_h - resized_h) // 2
        canvas = np.full((target_h, target_w, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + resized_h, pad_x:pad_x + resized_w] = resized
        return canvas, scale, pad_x, pad_y

    def detect(self, image) -> List[BoxDetection]:
        padded, scale, pad_x, pad_y = self._letterbox(image)
        blob = cv2.dnn.blobFromImage(padded, 1.0 / 255.0, self.input_size, swapRB=True, crop=False)
        self.net.setInput(blob)
        output = self.net.forward()
        rows = np.squeeze(output)
        if rows.ndim != 2:
            raise ValueError('unexpected detector output shape: %s' % (output.shape,))
        if rows.shape[0] < rows.shape[1] and rows.shape[0] in (4 + len(self.labels), 5 + len(self.labels)):
            rows = rows.T

        boxes = []
        confidences = []
        class_ids = []
        has_objectness = rows.shape[1] == 5 + len(self.labels)
        image_h, image_w = image.shape[:2]

        for row in rows:
            class_scores = row[5:] if has_objectness else row[4:]
            if not len(class_scores):
                continue
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])
            if has_objectness:
                confidence *= float(row[4])
            if confidence < self.confidence_threshold or class_id >= len(self.labels):
                continue

            center_x, center_y, box_w, box_h = [float(value) for value in row[:4]]
            left = int(round((center_x - box_w / 2.0 - pad_x) / scale))
            top = int(round((center_y - box_h / 2.0 - pad_y) / scale))
            width = int(round(box_w / scale))
            height = int(round(box_h / scale))
            left = max(0, min(left, image_w - 1))
            top = max(0, min(top, image_h - 1))
            width = max(1, min(width, image_w - left))
            height = max(1, min(height, image_h - top))
            boxes.append([left, top, width, height])
            confidences.append(confidence)
            class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.confidence_threshold, self.nms_threshold,
        )
        detections = []
        for index in np.array(indices).reshape(-1) if len(indices) else []:
            left, top, width, height = boxes[int(index)]
            detections.append(BoxDetection(
                class_id=class_ids[int(index)],
                confidence=confidences[int(index)],
                bbox=(left, top, left + width, top + height),
            ))
        return detections


def estimate_axis_angle(image, bbox) -> float:
    """Estimate the longest visible contour axis, in image-plane radians."""
    x1, y1, x2, y2 = bbox
    crop = image[y1:y2, x1:x2]
    if crop.size == 0 or min(crop.shape[:2]) < 8:
        return 0.0
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 40, 120)
    edges = cv2.morphologyEx(
        edges, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
    )
    contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    minimum_area = crop.shape[0] * crop.shape[1] * 0.01
    contours = [contour for contour in contours if cv2.contourArea(contour) >= minimum_area]
    if not contours:
        return 0.0
    rectangle = cv2.minAreaRect(max(contours, key=cv2.contourArea))
    corners = cv2.boxPoints(rectangle)
    vectors = [corners[(index + 1) % 4] - corners[index] for index in range(4)]
    longest = max(vectors, key=lambda vector: float(np.dot(vector, vector)))
    return float(np.arctan2(longest[1], longest[0]))
