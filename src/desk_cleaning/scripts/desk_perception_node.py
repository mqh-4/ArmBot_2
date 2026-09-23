#!/usr/bin/env python3
import hashlib
import math
import os
import threading
import time

import cv2
import message_filters
import rospy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

from desk_cleaning.core import PinholeProjector, sphere_center_depth
from desk_cleaning.depth import depth_median
from desk_cleaning.msg import DetectedObject, DetectedObjectArray
from desk_cleaning.yolo_onnx import (
    BoxDetection,
    YoloOnnxDetector,
    estimate_axis_angle,
    offset_bbox,
    roi_bounds,
)


class DeskPerceptionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.camera_info = None
        camera = rospy.get_param('~camera', {})
        model = rospy.get_param('~model', {})
        self.target_frame = camera.get('target_frame', 'base_link')
        self.depth_scale = float(camera.get('depth_scale', 0.001))
        self.depth_min = float(camera.get('depth_min', 0.12))
        self.depth_max = float(camera.get('depth_max', 1.20))
        self.depth_roi_ratio = float(camera.get('depth_roi_ratio', 0.25))
        self.fruit_depth_recovery_ratio = float(
            camera.get('fruit_depth_recovery_ratio', 1.0),
        )
        self.center_estimation = camera.get('center_estimation', {})
        self.roi_enabled = bool(camera.get('roi_enabled', True))
        self.desktop_roi = camera.get('desktop_roi', {
            'x_min': 0.05,
            'y_min': 0.20,
            'x_max': 0.95,
            'y_max': 0.98,
        })
        self.fallback_red_apple = bool(model.get('fallback_red_apple', True))
        self.fallback_confidence = min(float(model.get('fallback_confidence', 0.50)), 0.50)
        marker = camera.get('box_marker', {})
        self.box_marker_enabled = bool(marker.get('enabled', False))
        self.box_marker_name = str(marker.get('class_name', 'apple_box'))
        self.box_marker_mode = str(marker.get('mode', 'red_boundary'))
        self.box_marker_min_area = float(marker.get('min_area_px', 80.0))
        self.box_marker_max_area = float(marker.get('max_area_px', 12000.0))
        self.box_marker_min_circularity = float(marker.get('min_circularity', 0.72))
        self.box_marker_min_radius_px = float(marker.get('min_radius_px', 5.0))
        self.box_boundary_min_width_px = float(marker.get('min_width_px', 80.0))
        self.box_boundary_min_height_px = float(marker.get('min_height_px', 60.0))
        self.box_boundary_partial_enabled = bool(marker.get('partial_edge_enabled', True))
        self.box_boundary_wall_side = str(marker.get('wall_side', 'right')).strip().lower()
        self.box_boundary_partial_min_length_px = float(
            marker.get('partial_edge_min_length_px', 70.0),
        )
        self.box_boundary_partial_max_width_px = float(
            marker.get('partial_edge_max_width_px', 28.0),
        )
        self.box_boundary_partial_margin_px = int(
            marker.get('partial_edge_margin_px', 8),
        )
        self.box_boundary_partial_max_jump_px = float(
            marker.get('partial_edge_max_jump_px', 30.0),
        )
        self.box_boundary_partial_hold_sec = float(
            marker.get('partial_edge_hold_sec', 0.8),
        )
        self.box_boundary_partial_smoothing = float(
            marker.get('partial_edge_smoothing', 0.20),
        )
        self.partial_boundary_state = None
        self.complete_boundary_state = None
        self.labels = model.get('labels', [])
        self.display_names = rospy.get_param('~display_names', {})
        self.min_frame_interval = 1.0 / max(float(model.get('max_fps', 5.0)), 0.1)
        self.last_processed = 0.0
        cv2.setNumThreads(max(1, int(model.get('opencv_threads', 2))))

        model_path = os.path.expanduser(model.get('path', ''))
        # Keep an existing yolov8n deployment usable while switching the
        # default configuration to yolov8s: both files commonly live together.
        if (model_path and not os.path.isfile(model_path)
                and os.path.basename(model_path) == 'yolov8s.onnx'):
            fallback_path = os.path.join(os.path.dirname(model_path), 'yolov8n.onnx')
            if os.path.isfile(fallback_path):
                rospy.logwarn('yolov8s model not found, falling back to %s', fallback_path)
                model_path = fallback_path
        self.detector = None
        try:
            self.detector = YoloOnnxDetector(
                model_path=model_path,
                labels=self.labels,
                input_size=(
                    int(model.get('input_width', 640)),
                    int(model.get('input_height', 640)),
                ),
                confidence_threshold=float(model.get('confidence_threshold', 0.55)),
                nms_threshold=float(model.get('nms_threshold', 0.45)),
            )
            rospy.loginfo('desk perception loaded model: %s', model_path)
        except Exception as exc:
            rospy.logerr('desk perception has no usable ONNX model: %s', exc)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.objects_pub = rospy.Publisher('~objects', DetectedObjectArray, queue_size=1)
        self.debug_pub = rospy.Publisher('~debug_image', Image, queue_size=1)
        self.ready_pub = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)
        self.ready_pub.publish(Bool(data=False))

        color_topic = camera.get('color_topic', '/camera/color/image_raw')
        depth_topic = camera.get('depth_topic', '/camera/depth/image_raw')
        info_topic = camera.get('camera_info_topic', '/camera/color/camera_info')
        rospy.Subscriber(info_topic, CameraInfo, self.info_callback, queue_size=1)
        color_sub = message_filters.Subscriber(color_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.08,
        )
        sync.registerCallback(self.image_callback)
        self.sync = sync

    def info_callback(self, msg):
        with self.lock:
            self.camera_info = msg

    @staticmethod
    def stable_id(class_name, position):
        quantized = ':'.join(str(int(round(value / 0.02))) for value in position)
        digest = hashlib.sha1((class_name + ':' + quantized).encode('utf-8')).hexdigest()[:10]
        return class_name + '-' + digest

    def transform_point(self, point, source_frame, stamp):
        if source_frame == self.target_frame:
            return point
        stamped = PointStamped()
        stamped.header.frame_id = source_frame
        stamped.header.stamp = stamp
        stamped.point.x, stamped.point.y, stamped.point.z = point
        transform = self.tf_buffer.lookup_transform(
            self.target_frame, source_frame, stamp, rospy.Duration(0.08),
        )
        transformed = tf2_geometry_msgs.do_transform_point(stamped, transform)
        return transformed.point.x, transformed.point.y, transformed.point.z

    def fallback_red_apple_box(self, image):
        """Find a conservative red, roughly round candidate when YOLO misses it.

        This is a conservative fallback for a small round apple. Red boundary
        tape is filtered separately and cannot become this candidate.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = cv2.inRange(hsv, (0, 70, 45), (12, 255, 255))
        upper_red = cv2.inRange(hsv, (168, 70, 45), (180, 255, 255))
        mask = cv2.morphologyEx(
            cv2.bitwise_or(lower_red, upper_red), cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ( nine := 9, nine)),
        )
        image_h, image_w = image.shape[:2]
        candidates = []
        for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
            area = float(cv2.contourArea(contour))
            if area < 250.0 or area > image_w * image_h * 0.04:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * 3.141592653589793 * area / (perimeter * perimeter)
            x, y, w, h = cv2.boundingRect(contour)
            if (y < int(image_h * 0.25) or min(w, h) < 28
                    or max(w, h) > int(min(image_w, image_h) * 0.45)):
                continue
            aspect = float(w) / float(h)
            if circularity < 0.35 or aspect < 0.55 or aspect > 1.8:
                continue
            candidates.append((area * circularity, (x, y, x + w, y + h)))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0])[1]

    def box_marker_circle(self, image):
        """Find a dark, round box-centre marker in the configured desk ROI.

        This is deliberately conservative and only produces a visual marker
        detection when explicitly enabled.  The executor does not treat it as
        an apple, so a false marker cannot trigger a grasp.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 0, 65)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        candidates = []
        for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
            area = float(cv2.contourArea(contour))
            if not self.box_marker_min_area <= area <= self.box_marker_max_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.box_marker_min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius < self.box_marker_min_radius_px:
                continue
            candidates.append((area * circularity, (cx, cy, radius)))
        return max(candidates, key=lambda item: item[0])[1] if candidates else None

    def _smooth_boundary_detail(self, previous, current, alpha):
        detail = dict(current)
        for key in ('bbox', 'depth_bbox'):
            old_values = previous.get(key, detail[key])
            new_values = detail[key]
            detail[key] = tuple(
                int(round((1.0 - alpha) * old + alpha * new))
                for old, new in zip(old_values, new_values)
            )
        return detail

    def _stabilize_complete_boundary(self, candidates):
        now = time.monotonic()
        state = self.complete_boundary_state
        selected = None
        if state and now - state['seen'] <= self.box_boundary_partial_hold_sec:
            previous_bbox = state['details']['bbox']
            previous_center = (
                0.5 * (previous_bbox[0] + previous_bbox[2]),
                0.5 * (previous_bbox[1] + previous_bbox[3]),
            )
            nearby = []
            for candidate in candidates:
                bbox = candidate[1]['bbox']
                center = (0.5 * (bbox[0] + bbox[2]), 0.5 * (bbox[1] + bbox[3]))
                distance = math.hypot(center[0] - previous_center[0], center[1] - previous_center[1])
                if distance <= self.box_boundary_partial_max_jump_px:
                    nearby.append(candidate)
            if nearby:
                selected = max(nearby, key=lambda item: item[0])
                selected = (
                    selected[0],
                    self._smooth_boundary_detail(
                        state['details'], selected[1], self.box_boundary_partial_smoothing,
                    ),
                )
            else:
                return state['details']
        if selected is None:
            selected = max(candidates, key=lambda item: item[0])
        detail = selected[1]
        self.complete_boundary_state = {
            'seen': now,
            'details': detail,
        }
        return detail

    def box_boundary_details(self, image):
        """Find a full red outline or a visible edge of the placement box.

        The wall can hide the other two tape edges from the RGB camera.  In
        that case a long red edge is expanded from the edge toward the
        configured wall side and is used as a conservative no-grasp region.
        The original edge remains the depth sample used for the marker.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 70, 45), (12, 255, 255)),
            cv2.inRange(hsv, (168, 70, 45), (180, 255, 255)),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)),
        )
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        complete_candidates = []
        candidates = []
        image_height, image_width = image.shape[:2]

        def wall_bbox(axis, edge):
            if axis == 'vertical':
                edge = int(round(max(0.0, min(float(image_width), edge))))
                if self.box_boundary_wall_side == 'left':
                    return (0, 0, edge, image_height)
                return (edge, 0, image_width, image_height)
            edge = int(round(max(0.0, min(float(image_height), edge))))
            if self.box_boundary_wall_side == 'top':
                return (0, 0, image_width, edge)
            return (0, edge, image_width, image_height)
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if width < self.box_boundary_min_width_px or height < self.box_boundary_min_height_px:
                continue
            aspect = float(width) / float(height)
            if aspect < 0.35 or aspect > 3.0:
                continue
            region = mask[y:y + height, x:x + width]
            fill_ratio = float(cv2.countNonZero(region)) / float(max(1, width * height))
            if fill_ratio < 0.015 or fill_ratio > 0.45:
                continue
            edge_width = max(2, min(12, int(round(min(width, height) * 0.08))))
            edge_pixels = cv2.vconcat((
                region[:edge_width, :], region[-edge_width:, :],
            ))
            side_pixels = cv2.hconcat((
                region[:, :edge_width], region[:, -edge_width:],
            ))
            edge_ratio = (
                float(cv2.countNonZero(edge_pixels)) +
                float(cv2.countNonZero(side_pixels))
            ) / float(max(1, edge_pixels.size + side_pixels.size))
            if edge_ratio < 0.12:
                continue
            bbox = (x, y, x + width, y + height)
            complete_candidates.append((width * height * edge_ratio, {
                'bbox': bbox,
                'depth_bbox': bbox,
                'partial': False,
            }))

        if self.box_boundary_partial_enabled:
            # Hough lines still recover a tape edge when the red contour is
            # split by the gripper or merged with a red apple.  The region is
            # deliberately extended across the whole image toward the wall;
            # this is a safety exclusion, not a precise box outline.
            line_edges = cv2.Canny(mask, 50, 150)
            lines = cv2.HoughLinesP(
                line_edges, 1, math.pi / 180.0,
                threshold=max(20, int(self.box_boundary_partial_min_length_px * 0.45)),
                minLineLength=int(self.box_boundary_partial_min_length_px),
                maxLineGap=max(10, int(self.box_boundary_partial_min_length_px * 0.2)),
            )
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = [int(value) for value in line[0]]
                    dx, dy = x2 - x1, y2 - y1
                    line_length = math.hypot(dx, dy)
                    if line_length < self.box_boundary_partial_min_length_px:
                        continue
                    vertical = abs(dx) <= max(2.0, abs(dy) * 0.15)
                    horizontal = abs(dy) <= max(2.0, abs(dx) * 0.15)
                    if not (vertical or horizontal):
                        continue
                    margin = max(2, self.box_boundary_partial_margin_px)
                    depth_bbox = (
                        max(0, min(x1, x2) - margin),
                        max(0, min(y1, y2) - margin),
                        min(image_width, max(x1, x2) + margin),
                        min(image_height, max(y1, y2) + margin),
                    )
                    if vertical:
                        edge_x = int(round((x1 + x2) * 0.5))
                        axis, edge = 'vertical', edge_x
                    else:
                        edge_y = int(round((y1 + y2) * 0.5))
                        axis, edge = 'horizontal', edge_y
                    bbox = wall_bbox(axis, edge)
                    candidates.append((line_length * line_length, {
                        'bbox': bbox,
                        'depth_bbox': depth_bbox,
                        'partial': True,
                        'axis': axis,
                        'edge': float(edge),
                    }))

            for contour in contours:
                x, y, width, height = cv2.boundingRect(contour)
                vertical = height >= self.box_boundary_partial_min_length_px and (
                    width <= self.box_boundary_partial_max_width_px
                )
                horizontal = width >= self.box_boundary_partial_min_length_px and (
                    height <= self.box_boundary_partial_max_width_px
                )
                if not (vertical or horizontal):
                    continue
                line_length = max(width, height)
                if cv2.countNonZero(mask[y:y + height, x:x + width]) < line_length * 2.0:
                    continue
                margin = max(2, self.box_boundary_partial_margin_px)
                depth_bbox = (
                    max(0, x - margin), max(0, y - margin),
                    min(image_width, x + width + margin),
                    min(image_height, y + height + margin),
                )
                if vertical:
                    axis, edge = 'vertical', x + 0.5 * width
                else:
                    axis, edge = 'horizontal', y + 0.5 * height
                bbox = wall_bbox(axis, edge)
                candidates.append((line_length * line_length, {
                    'bbox': bbox,
                    'depth_bbox': depth_bbox,
                    'partial': True,
                    'axis': axis,
                    'edge': float(edge),
                }))
        if complete_candidates:
            self.partial_boundary_state = None
            return self._stabilize_complete_boundary(complete_candidates)
        if self.complete_boundary_state:
            state = self.complete_boundary_state
            if time.monotonic() - state['seen'] <= self.box_boundary_partial_hold_sec:
                self.partial_boundary_state = None
                return state['details']
            self.complete_boundary_state = None
        if not candidates:
            state = self.partial_boundary_state
            if state and time.monotonic() - state['seen'] <= self.box_boundary_partial_hold_sec:
                return state['details']
            self.partial_boundary_state = None
            return None

        now = time.monotonic()
        state = self.partial_boundary_state
        selected = None
        if state and now - state['seen'] <= self.box_boundary_partial_hold_sec:
            nearby = [
                candidate for candidate in candidates
                if candidate[1].get('axis') == state['axis']
                and abs(candidate[1].get('edge', 0.0) - state['edge'])
                <= self.box_boundary_partial_max_jump_px
            ]
            if nearby:
                selected = max(nearby, key=lambda item: item[0])
                raw_detail = selected[1]
                smooth = (
                    (1.0 - self.box_boundary_partial_smoothing) * state['edge']
                    + self.box_boundary_partial_smoothing * raw_detail['edge']
                )
                detail = dict(raw_detail)
                detail['edge'] = smooth
                detail['bbox'] = wall_bbox(detail['axis'], smooth)
                selected = (selected[0], detail)
            else:
                return state['details']
        if selected is None:
            selected = max(candidates, key=lambda item: item[0])
        detail = selected[1]
        self.partial_boundary_state = {
            'axis': detail['axis'],
            'edge': float(detail['edge']),
            'seen': now,
            'details': detail,
        }
        return detail

    def box_boundary(self, image):
        """Return the detected boundary bbox for compatibility with callers."""
        details = self.box_boundary_details(image)
        return details['bbox'] if details is not None else None

    def image_callback(self, color_msg, depth_msg):
        now = time.monotonic()
        if now - self.last_processed < self.min_frame_interval:
            return
        self.last_processed = now
        with self.lock:
            camera_info = self.camera_info
        output = DetectedObjectArray()
        output.header.stamp = color_msg.header.stamp
        output.header.frame_id = self.target_frame
        if camera_info is None or self.detector is None:
            self.objects_pub.publish(output)
            return

        try:
            source_frame = color_msg.header.frame_id or camera_info.header.frame_id
            if not source_frame:
                raise ValueError('camera image and CameraInfo frame_id are both empty')
            if source_frame != self.target_frame and not self.tf_buffer.can_transform(
                self.target_frame, source_frame, color_msg.header.stamp, rospy.Duration(0.08),
            ):
                self.ready_pub.publish(Bool(data=False))
                self.objects_pub.publish(output)
                rospy.logwarn_throttle(2.0, 'waiting for camera-to-base TF')
                return
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            projector = PinholeProjector(
                camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5],
            )
            if self.roi_enabled:
                roi_x1, roi_y1, roi_x2, roi_y2 = roi_bounds(color.shape, self.desktop_roi)
                detector_image = color[roi_y1:roi_y2, roi_x1:roi_x2]
            else:
                roi_x1, roi_y1 = 0, 0
                detector_image = color

            detections = self.detector.detect(detector_image)
            if roi_x1 or roi_y1:
                detections = [
                    BoxDetection(box.class_id, box.confidence,
                                 offset_bbox(box.bbox, roi_x1, roi_y1))
                    for box in detections
                ]
            for box in detections:
                class_name = self.labels[box.class_id]
                if class_name in ('apple', 'orange', 'banana'):
                    rospy.loginfo_throttle(
                        2.0,
                        'raw fruit detection class=%s confidence=%.3f bbox=%s',
                        class_name, box.confidence, box.bbox,
                    )
            # A model apple box with invalid depth must not suppress the
            # fallback.  Only a model box that overlaps the red candidate is
            # authoritative; unrelated apple/orange false positives are not.
            boundary_details = (
                self.box_boundary_details(detector_image)
                if self.box_marker_enabled and self.box_marker_mode == 'red_boundary'
                else None
            )
            box_boundary = boundary_details['bbox'] if boundary_details is not None else None
            fallback_box = (
                self.fallback_red_apple_box(detector_image)
                if self.fallback_red_apple and 'apple' in self.labels and box_boundary is None
                else None
            )
            model_apple_overlaps = False
            if fallback_box is not None:
                fx1, fy1, fx2, fy2 = fallback_box
                farea = max(1, (fx2 - fx1) * (fy2 - fy1))
                for model_box in detections:
                    if self.labels[model_box.class_id] != 'apple':
                        continue
                    mx1, my1, mx2, my2 = model_box.bbox
                    ix1, iy1 = max(fx1, mx1), max(fy1, my1)
                    ix2, iy2 = min(fx2, mx2), min(fy2, my2)
                    intersection = max(0, ix2 - ix1) * max(0, iy2 - iy1)
                    if intersection / float(farea) >= 0.25:
                        model_apple_overlaps = True
                        break
            if fallback_box is not None and not model_apple_overlaps:
                rospy.loginfo_throttle(
                    2.0, 'red apple fallback bbox=%s confidence=%.2f',
                    fallback_box, self.fallback_confidence,
                )
                detections.append(BoxDetection(
                    class_id=self.labels.index('apple'),
                    confidence=self.fallback_confidence,
                    bbox=offset_bbox(fallback_box, roi_x1, roi_y1),
                ))
            debug_image = color.copy()
            if self.roi_enabled:
                cv2.rectangle(debug_image, (roi_x1, roi_y1), (roi_x2 - 1, roi_y2 - 1),
                              (255, 180, 0), 2)
            if self.box_marker_enabled:
                if self.box_marker_mode == 'red_boundary':
                    marker = box_boundary
                    if marker is not None:
                        marker_bbox = tuple(
                            int(value + offset) for value, offset in zip(
                                marker, (roi_x1, roi_y1, roi_x1, roi_y1),
                            )
                        )
                        marker_x = 0.5 * (marker_bbox[0] + marker_bbox[2])
                        marker_y = 0.5 * (marker_bbox[1] + marker_bbox[3])
                        marker_radius = 0.5 * max(
                            marker_bbox[2] - marker_bbox[0],
                            marker_bbox[3] - marker_bbox[1],
                        )
                        marker_depth_bbox = tuple(
                            int(value + offset) for value, offset in zip(
                                boundary_details['depth_bbox'],
                                (roi_x1, roi_y1, roi_x1, roi_y1),
                            )
                        )
                        marker_partial = bool(boundary_details.get('partial', False))
                else:
                    marker = self.box_marker_circle(detector_image)
                    if marker is not None:
                        marker_x, marker_y, marker_radius = marker
                        marker_x += roi_x1
                        marker_y += roi_y1
                        marker_bbox = (
                            int(round(marker_x - marker_radius)),
                            int(round(marker_y - marker_radius)),
                            int(round(marker_x + marker_radius)),
                            int(round(marker_y + marker_radius)),
                        )
                        marker_depth_bbox = marker_bbox
                        marker_partial = False
                if marker is not None:
                    marker_depth = depth_median(
                        depth, marker_depth_bbox, self.depth_scale, self.depth_min,
                        self.depth_max, max(self.depth_roi_ratio, 0.75),
                    )
                    if marker_depth is not None:
                        mu, mv, marker_z = marker_depth
                        marker_point = projector.project(mu, mv, marker_z)
                        try:
                            marker_base = self.transform_point(
                                marker_point, source_frame, color_msg.header.stamp,
                            )
                            marker_item = DetectedObject()
                            marker_item.id = self.stable_id(self.box_marker_name, marker_base)
                            marker_item.class_name = self.box_marker_name
                            marker_item.display_name = self.display_names.get(
                                self.box_marker_name, '盒子中心',
                            )
                            # A partial edge is reliable for exclusion but not
                            # for estimating the box centre used for placement.
                            marker_item.confidence = 0.5 if marker_partial else 1.0
                            marker_item.x_min, marker_item.y_min, marker_item.x_max, marker_item.y_max = marker_bbox
                            marker_item.position.x, marker_item.position.y, marker_item.position.z = marker_base
                            marker_item.size_x = (marker_bbox[2] - marker_bbox[0]) * marker_z / projector.fx
                            marker_item.size_y = (marker_bbox[3] - marker_bbox[1]) * marker_z / projector.fy
                            marker_item.valid_depth = True
                            output.objects.append(marker_item)
                            if self.box_marker_mode == 'red_boundary':
                                cv2.rectangle(
                                    debug_image,
                                    (marker_bbox[0], marker_bbox[1]),
                                    (marker_bbox[2], marker_bbox[3]),
                                    (255, 0, 255), 2,
                                )
                            else:
                                cv2.circle(
                                    debug_image, (int(round(marker_x)), int(round(marker_y))),
                                    int(round(marker_radius)), (255, 0, 255), 2,
                                )
                            cv2.putText(
                                debug_image,
                                '%s x=%.3f y=%.3f z=%.3f' % (
                                    self.box_marker_name, marker_base[0], marker_base[1], marker_base[2],
                                ),
                                (int(round(marker_x)) + 8, int(round(marker_y))),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA,
                            )
                        except Exception as exc:
                            rospy.logwarn_throttle(2.0, 'box marker TF error: %s', exc)
            for box in detections:
                bbox = box.bbox
                class_name = self.labels[box.class_id]
                depth_sample = depth_median(
                    depth, bbox, self.depth_scale, self.depth_min, self.depth_max,
                    self.depth_roi_ratio,
                )
                # Astra depth can have an invalid hole at a fruit center. First
                # expand to the inner 75%; if that is still empty, use the
                # full fruit bbox as a last recovery step. This keeps the
                # normal center sample conservative while surviving local
                # depth dropouts near the bottom of the image.
                if depth_sample is None and class_name in ('apple', 'orange', 'banana'):
                    depth_sample = depth_median(
                        depth, bbox, self.depth_scale, self.depth_min,
                        self.depth_max, max(self.depth_roi_ratio, 0.75),
                    )
                if depth_sample is None and class_name in ('apple', 'orange', 'banana'):
                    depth_sample = depth_median(
                        depth, bbox, self.depth_scale, self.depth_min,
                        self.depth_max, max(self.fruit_depth_recovery_ratio, 1.0),
                    )
                    if depth_sample is not None:
                        rospy.loginfo_throttle(
                            2.0,
                            '%s bbox=%s recovered depth from full fruit bbox',
                            class_name, bbox,
                        )
                if depth_sample is None:
                    if class_name in ('apple', 'orange', 'banana'):
                        rospy.logwarn_throttle(
                            2.0, '%s bbox=%s rejected: no valid depth', class_name, bbox,
                        )
                    continue
                u, v, depth_m = depth_sample
                # Center-pixel depth on a round object measures its front
                # surface. Move one apparent radius along the optical ray to
                # estimate the object's geometric center.
                center_mode = self.center_estimation.get(class_name, '')
                object_depth = depth_m
                if center_mode == 'sphere_bbox':
                    object_depth = sphere_center_depth(
                        depth_m, bbox, projector.fx, projector.fy,
                    )
                elif center_mode == 'thin_bbox':
                    # A banana is elongated in the image but thin through
                    # its grasping diameter. Estimate its 3-D centre from
                    # half of the short projected dimension, not half of its
                    # long length.
                    width_m = (bbox[2] - bbox[0]) * depth_m / projector.fx
                    height_m = (bbox[3] - bbox[1]) * depth_m / projector.fy
                    object_depth = depth_m + 0.5 * min(width_m, height_m)
                camera_point = projector.project(u, v, object_depth)
                image_angle = estimate_axis_angle(color, bbox)
                try:
                    base_point = self.transform_point(
                        camera_point, source_frame, color_msg.header.stamp,
                    )
                    axis_camera_point = projector.project(
                        u + 20.0 * math.cos(image_angle),
                        v + 20.0 * math.sin(image_angle),
                        object_depth,
                    )
                    axis_base_point = self.transform_point(
                        axis_camera_point, source_frame, color_msg.header.stamp,
                    )
                except Exception as exc:
                    if class_name in ('apple', 'orange', 'banana'):
                        rospy.logwarn_throttle(
                            2.0, '%s bbox=%s rejected: RGB-D TF error: %s',
                            class_name, bbox, exc,
                        )
                    rospy.logwarn_throttle(2.0, 'RGB-D TF unavailable: %s', exc)
                    continue

                item = DetectedObject()
                item.id = self.stable_id(class_name, base_point)
                item.class_name = class_name
                item.display_name = self.display_names.get(class_name, class_name)
                item.confidence = box.confidence
                item.x_min, item.y_min, item.x_max, item.y_max = bbox
                item.position.x, item.position.y, item.position.z = base_point
                item.grasp_yaw = math.atan2(
                    axis_base_point[1] - base_point[1],
                    axis_base_point[0] - base_point[0],
                )
                item.size_x = (bbox[2] - bbox[0]) * object_depth / projector.fx
                item.size_y = (bbox[3] - bbox[1]) * object_depth / projector.fy
                item.size_z = 0.0
                item.valid_depth = True
                output.objects.append(item)
                cv2.rectangle(debug_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                              (0, 220, 0), 2)
                cv2.putText(debug_image, '%s %.2f' % (class_name, box.confidence),
                            (bbox[0], max(15, bbox[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 220, 0), 1, cv2.LINE_AA)
            self.objects_pub.publish(output)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))
            self.ready_pub.publish(Bool(data=True))
        except Exception as exc:
            self.ready_pub.publish(Bool(data=False))
            rospy.logerr_throttle(2.0, 'desk perception frame failed: %s', exc)


if __name__ == '__main__':
    rospy.init_node('desk_perception')
    DeskPerceptionNode()
    rospy.spin()
