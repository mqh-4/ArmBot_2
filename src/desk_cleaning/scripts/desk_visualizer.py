#!/usr/bin/env python3
"""Local OpenCV viewer for the desk-cleaning RGB-D perception topics."""

import os
import threading
import time
import tkinter as tk

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from desk_cleaning.core import Workspace
from desk_cleaning.msg import DetectedObjectArray


class DeskVisualizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.latest_image = None
        self.latest_objects = []
        self.objects_received_at = 0.0
        self.last_frame_at = None
        self.smoothed_fps = 0.0
        self.last_display = None

        self.window_name = rospy.get_param('~window_name', 'ArmPi-FPV Camera')
        self.window_width = int(rospy.get_param('~window_width', 960))
        self.window_height = int(rospy.get_param('~window_height', 720))
        # The detector is intentionally throttled on the Pi. Keep the last
        # non-empty result briefly when an inference misses one frame.
        self.objects_timeout = float(rospy.get_param('~objects_timeout', 8.0))
        self.show_guide = bool(rospy.get_param('~show_guide', True))
        self.snapshot_path = os.path.expanduser(rospy.get_param(
            '~snapshot_path',
            '/home/ubuntu/share/tmp/ArmPi-FPV/output/visualizer_snapshot.jpg',
        ))
        self.target_classes = set(rospy.get_param(
            '~target_classes', ['toothbrush', 'spoon', 'apple', 'orange', 'banana'],
        ))
        workspace = rospy.get_param('~workspace', {})
        self.workspace = Workspace(
            x_min=float(workspace.get('x_min', 0.10)),
            x_max=float(workspace.get('x_max', 0.28)),
            y_min=float(workspace.get('y_min', -0.16)),
            y_max=float(workspace.get('y_max', 0.16)),
            z_min=float(workspace.get('z_min', -0.01)),
            z_max=float(workspace.get('z_max', 0.10)),
            radial_reach=workspace.get('radial_reach', ()),
        )

        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        objects_topic = rospy.get_param('~objects_topic', '/desk_perception/objects')
        rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24,
        )
        rospy.Subscriber(
            objects_topic, DetectedObjectArray, self.objects_callback, queue_size=1,
        )
        rospy.loginfo('desk visualizer image topic: %s', image_topic)
        rospy.loginfo('desk visualizer objects topic: %s', objects_topic)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'visualizer image conversion failed: %s', exc)
            return
        now = time.monotonic()
        with self.lock:
            if self.last_frame_at is not None:
                interval = now - self.last_frame_at
                if interval > 0.0:
                    instant_fps = 1.0 / interval
                    if self.smoothed_fps <= 0.0:
                        self.smoothed_fps = instant_fps
                    else:
                        self.smoothed_fps = self.smoothed_fps * 0.9 + instant_fps * 0.1
            self.last_frame_at = now
            self.latest_image = image.copy()

    def objects_callback(self, msg):
        with self.lock:
            if msg.objects:
                self.latest_objects = list(msg.objects)
                self.objects_received_at = time.monotonic()

    @staticmethod
    def draw_text(image, text, origin, color=(255, 255, 255), scale=0.55):
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 1
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, scale, thickness,
        )
        x, y = origin
        x = max(0, min(x, image.shape[1] - text_width - 6))
        y = max(text_height + 6, min(y, image.shape[0] - baseline - 3))
        cv2.rectangle(
            image,
            (x, y - text_height - 6),
            (x + text_width + 6, y + baseline + 3),
            (20, 20, 20),
            -1,
        )
        cv2.putText(
            image, text, (x + 3, y - 3), font, scale, color,
            thickness, cv2.LINE_AA,
        )

    def draw_overlay(self, image, objects, fps):
        height, width = image.shape[:2]
        if self.show_guide:
            guide_left = int(width * 0.25)
            guide_top = int(height * 0.20)
            guide_right = int(width * 0.75)
            guide_bottom = int(height * 0.80)
            cv2.rectangle(
                image, (guide_left, guide_top), (guide_right, guide_bottom),
                (255, 180, 0), 1, cv2.LINE_AA,
            )
            center = (width // 2, height // 2)
            cv2.drawMarker(
                image, center, (255, 180, 0), cv2.MARKER_CROSS, 24, 1,
                cv2.LINE_AA,
            )

        safe_targets = 0
        for item in objects:
            x1 = max(0, min(int(item.x_min), width - 1))
            y1 = max(0, min(int(item.y_min), height - 1))
            x2 = max(0, min(int(item.x_max), width - 1))
            y2 = max(0, min(int(item.y_max), height - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            is_marker = item.class_name == 'apple_box'
            is_target = item.class_name in self.target_classes
            in_workspace = item.valid_depth and self.workspace.contains((
                item.position.x, item.position.y, item.position.z,
            ))
            safe_target = is_target and in_workspace
            if safe_target:
                safe_targets += 1
            color = (255, 0, 255) if is_marker else (
                (40, 220, 40) if safe_target
                else (0, 180, 255) if is_target
                else (170, 170, 170)
            )
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
            label = '{} {:.2f}'.format(item.class_name, item.confidence)
            if item.valid_depth:
                label += ' x={:.2f} y={:.2f} z={:.2f} {}'.format(
                    item.position.x, item.position.y, item.position.z,
                    'SAFE' if in_workspace else 'OUT',
                )
            self.draw_text(image, label, (x1, y1 - 4), color=color)

        status_color = (40, 220, 40) if safe_targets else (0, 180, 255)
        self.draw_text(
            image,
            'FPS {:.1f} | detections {} | Q/Esc quit | S snapshot'.format(
                fps, len(objects),
            ),
            (8, 25),
            color=status_color,
            scale=0.5,
        )
        return image

    def save_snapshot(self, image):
        directory = os.path.dirname(self.snapshot_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        if cv2.imwrite(self.snapshot_path, image):
            rospy.loginfo('visualizer snapshot saved: %s', self.snapshot_path)
        else:
            rospy.logerr('failed to save visualizer snapshot: %s', self.snapshot_path)

    def run(self):
        try:
            root = tk.Tk()
            root.title(self.window_name)
            root.geometry('{}x{}'.format(self.window_width, self.window_height))
            root.attributes('-topmost', True)
            root.lift()
            root.focus_force()
            image_label = tk.Label(root, background='#1c1c1c', borderwidth=0)
            image_label.pack(fill=tk.BOTH, expand=True)
        except tk.TclError as exc:
            rospy.logfatal('cannot open visualizer window; check DISPLAY/X11: %s', exc)
            return

        def close_window():
            if not rospy.is_shutdown():
                rospy.signal_shutdown('visualizer window closed by user')

        def handle_key(event):
            key = event.keysym.lower()
            if key in ('q', 'escape'):
                close_window()
            elif key == 's' and self.last_display is not None:
                self.save_snapshot(self.last_display)

        def render_frame():
            if rospy.is_shutdown():
                try:
                    root.destroy()
                except tk.TclError:
                    pass
                return
            with self.lock:
                image = None if self.latest_image is None else self.latest_image.copy()
                fps = self.smoothed_fps
                age = time.monotonic() - self.objects_received_at
                objects = list(self.latest_objects) if age <= self.objects_timeout else []
            if image is None:
                display = np.full((480, 640, 3), 28, dtype=np.uint8)
                self.draw_text(
                    display,
                    'Waiting for /camera/color/image_raw ...',
                    (90, 235),
                    color=(0, 180, 255),
                    scale=0.7,
                )
            else:
                display = self.draw_overlay(image, objects, fps)
            self.last_display = display.copy()
            resized = cv2.resize(
                display, (self.window_width, self.window_height),
                interpolation=cv2.INTER_AREA,
            )
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            ppm = 'P6\n{} {}\n255\n'.format(
                self.window_width, self.window_height,
            ).encode('ascii') + rgb.tobytes()
            try:
                photo = tk.PhotoImage(data=ppm, format='PPM')
                image_label.configure(image=photo)
                image_label.image = photo
                root.after(50, render_frame)
            except tk.TclError as exc:
                rospy.logerr('visualizer window update failed: %s', exc)
                close_window()

        root.protocol('WM_DELETE_WINDOW', close_window)
        root.bind('<Key>', handle_key)
        root.after(0, render_frame)
        try:
            root.mainloop()
        finally:
            if not rospy.is_shutdown():
                rospy.signal_shutdown('visualizer main loop exited')


if __name__ == '__main__':
    rospy.init_node('desk_visualizer')
    DeskVisualizer().run()
