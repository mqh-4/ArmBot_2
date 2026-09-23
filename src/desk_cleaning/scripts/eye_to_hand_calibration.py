#!/usr/bin/env python3
"""Collect and solve eye-to-hand samples with a ChArUco target on the flange.

Collection is passive: this node never commands the robot. Move the arm with
the normal, separately verified controller and press S to save a valid pose.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rospy
import tf2_ros
import tkinter as tk
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur, ServoStateList


BOARD_SQUARES_X = 6
BOARD_SQUARES_Y = 4
BOARD_SQUARE_LENGTH_M = 0.020
BOARD_MARKER_LENGTH_M = 0.015
BOARD_DICTIONARY = cv2.aruco.DICT_4X4_50


def make_board():
    dictionary = cv2.aruco.getPredefinedDictionary(BOARD_DICTIONARY)
    if hasattr(cv2.aruco, 'CharucoBoard_create'):
        return cv2.aruco.CharucoBoard_create(
            BOARD_SQUARES_X, BOARD_SQUARES_Y,
            BOARD_SQUARE_LENGTH_M, BOARD_MARKER_LENGTH_M, dictionary,
        )
    return cv2.aruco.CharucoBoard(
        (BOARD_SQUARES_X, BOARD_SQUARES_Y),
        BOARD_SQUARE_LENGTH_M, BOARD_MARKER_LENGTH_M, dictionary,
    )


def rt_to_matrix(rotation, translation):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def matrix_to_dict(matrix):
    return np.asarray(matrix, dtype=np.float64).round(12).tolist()


def matrix_from_dict(value):
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('expected 4x4 transform, got {}'.format(matrix.shape))
    return matrix


def rotation_mean(rotations):
    """Project the sum of rotation matrices back onto SO(3)."""
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    result = u.dot(vt)
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u.dot(vt)
    return result


def mean_transform(transforms):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_mean([item[:3, :3] for item in transforms])
    result[:3, 3] = np.mean([item[:3, 3] for item in transforms], axis=0)
    return result


def rotation_error_degrees(rotation):
    value = max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0))
    return math.degrees(math.acos(value))


def solve_samples(samples):
    if len(samples) < 6:
        raise ValueError('at least 6 samples are required; collect 10-20')

    gripper_to_base = [matrix_from_dict(item['base_to_end']) for item in samples]
    camera_to_target = [matrix_from_dict(item['camera_to_target']) for item in samples]

    # Treat the moving ChArUco target as the "camera" and the fixed RGB-D
    # camera as the "target" in OpenCV's standard hand-eye equation. This
    # solves the unknown fixed target-to-gripper transform first.
    r_gripper2base = [item[:3, :3] for item in gripper_to_base]
    t_gripper2base = [item[:3, 3].reshape(3, 1) for item in gripper_to_base]
    target_to_camera = [np.linalg.inv(item) for item in camera_to_target]
    r_target2cam = [item[:3, :3] for item in target_to_camera]
    t_target2cam = [item[:3, 3].reshape(3, 1) for item in target_to_camera]

    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gripper2base, t_gripper2base,
        r_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    end_to_board = rt_to_matrix(r_cam2gripper, t_cam2gripper)

    # For every sample: base->end * end->board = base->camera * camera->board.
    base_to_camera_candidates = [
        base_to_end.dot(end_to_board).dot(np.linalg.inv(camera_to_target_i))
        for base_to_end, camera_to_target_i in zip(gripper_to_base, camera_to_target)
    ]
    base_to_camera = mean_transform(base_to_camera_candidates)

    residuals = []
    for base_to_end, camera_to_target_i in zip(gripper_to_base, camera_to_target):
        predicted = base_to_camera.dot(camera_to_target_i)
        actual = base_to_end.dot(end_to_board)
        error = np.linalg.inv(predicted).dot(actual)
        residuals.append({
            'translation_m': float(np.linalg.norm(error[:3, 3])),
            'rotation_deg': rotation_error_degrees(error[:3, :3]),
        })

    return {
        'base_to_camera': base_to_camera,
        'end_to_board': end_to_board,
        'residuals': residuals,
    }


def detect_board(image, board, camera_matrix, distortion, minimum_corners):
    dictionary = cv2.aruco.getPredefinedDictionary(BOARD_DICTIONARY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    if marker_ids is None or len(marker_ids) < 4:
        return None
    _, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, image, board,
    )
    if ids is None or len(ids) < minimum_corners:
        return None
    try:
        valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            corners, ids, board, camera_matrix, distortion, None, None,
        )
    except cv2.error:
        return None
    if not valid:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    camera_to_target = rt_to_matrix(rotation, tvec)
    return corners, ids, camera_to_target


class Collector:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.board = make_board()
        self.lock = threading.RLock()
        self.latest_image = None
        self.latest_stamp = None
        self.camera_matrix = None
        self.distortion = None
        self.pulses = {}
        self.motion_enabled = bool(rospy.get_param('~allow_motion', False))
        self.step_pulse = max(1, min(50, int(rospy.get_param('~step_pulse', 15))))
        self.motion_duration = max(0.3, min(3.0, float(rospy.get_param('~motion_duration', 1.0))))
        self.minimum_pulse = max(20, min(400, int(rospy.get_param('~minimum_pulse', 100))))
        self.maximum_pulse = min(980, max(600, int(rospy.get_param('~maximum_pulse', 900))))
        self.motion_pub = rospy.Publisher(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur, queue_size=1,
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.samples = self.load_samples() if args.mode == 'collect' else []
        self.base_to_camera = None
        self.end_to_board = None
        if args.mode == 'validate':
            with open(args.result, 'r', encoding='utf-8') as stream:
                result = json.load(stream)
            self.base_to_camera = matrix_from_dict(result['base_to_camera'])
            self.end_to_board = matrix_from_dict(result['end_to_board'])
        rospy.Subscriber(args.image_topic, Image, self.image_callback, queue_size=1)
        rospy.Subscriber(args.camera_info_topic, CameraInfo, self.info_callback, queue_size=1)
        rospy.Subscriber(
            '/servo_controllers/port_id_1/servo_states', ServoStateList,
            self.servo_state_callback, queue_size=1,
        )

    def servo_state_callback(self, message):
        for state in message.servo_states:
            if int(state.id) in (2, 3, 4, 5, 6):
                self.pulses[int(state.id)] = int(state.position)

    def load_samples(self):
        if not os.path.exists(self.args.samples):
            return []
        with open(self.args.samples, 'r', encoding='utf-8') as stream:
            payload = json.load(stream)
        return list(payload.get('samples', []))

    def save_samples(self):
        directory = os.path.dirname(self.args.samples)
        if directory:
            os.makedirs(directory, exist_ok=True)
        payload = {
            'board': {
                'squares_x': BOARD_SQUARES_X,
                'squares_y': BOARD_SQUARES_Y,
                'square_length_m': BOARD_SQUARE_LENGTH_M,
                'marker_length_m': BOARD_MARKER_LENGTH_M,
                'dictionary': 'DICT_4X4_50',
            },
            'base_frame': self.args.base_frame,
            'end_frame': self.args.end_frame,
            'image_topic': self.args.image_topic,
            'samples': self.samples,
        }
        with open(self.args.samples, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2)

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'image conversion failed: %s', exc)
            return
        with self.lock:
            self.latest_image = image.copy()
            self.latest_stamp = message.header.stamp

    def info_callback(self, message):
        with self.lock:
            self.camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
            self.distortion = np.asarray(message.D, dtype=np.float64)

    def publish_motion(self, positions, duration):
        if not self.motion_enabled:
            rospy.logwarn('motion disabled; no servo command was sent')
            return
        self.motion_pub.publish(MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(servo_id), int(position), float(duration))
                for servo_id, position in sorted(positions.items())
            ],
        ))

    def stop_motion(self):
        if self.pulses:
            self.publish_motion(dict(self.pulses), 0.15)

    def jog_motion(self, servo_id, direction, label):
        if servo_id not in self.pulses:
            rospy.logwarn('waiting for servo %d state feedback', servo_id)
            return
        target = self.pulses[servo_id] + direction * self.step_pulse
        target = max(self.minimum_pulse, min(self.maximum_pulse, target))
        if target == self.pulses[servo_id]:
            rospy.logwarn('servo %d is at a safety limit', servo_id)
            return
        self.pulses[servo_id] = target
        rospy.loginfo('%s: servo %d -> %d', label, servo_id, target)
        self.publish_motion({servo_id: target}, self.motion_duration)

    def robot_transform(self, stamp):
        transform = self.tf_buffer.lookup_transform(
            self.args.base_frame, self.args.end_frame, stamp, rospy.Duration(0.2),
        )
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        rotation = cv2.Rodrigues(np.zeros((3, 1), dtype=np.float64))[0]
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        return rt_to_matrix(rotation, (translation.x, translation.y, translation.z))

    def draw(self, image, detection, message):
        output = image.copy()
        if detection is None:
            cv2.putText(output, 'ChArUco not detected', (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 255), 2)
        else:
            corners, ids, camera_to_target = detection
            cv2.aruco.drawDetectedCornersCharuco(output, corners, ids)
            cv2.drawFrameAxes(
                output, self.camera_matrix, self.distortion,
                cv2.Rodrigues(camera_to_target[:3, :3])[0],
                camera_to_target[:3, 3], 0.05, 2,
            )
            if self.args.mode == 'validate':
                label = 'ChArUco {} corners | validation | Q/Esc quit'.format(len(ids))
            else:
                label = 'ChArUco {} corners | Shift+S save | Q/Esc quit'.format(len(ids))
            cv2.putText(output, label, (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 220, 40), 2)
            if self.args.mode == 'validate' and message is not None:
                try:
                    base_to_end = self.robot_transform(message)
                    board_from_camera = self.base_to_camera.dot(camera_to_target)
                    board_from_robot = base_to_end.dot(self.end_to_board)
                    error = np.linalg.inv(board_from_camera).dot(board_from_robot)
                    translation_mm = np.linalg.norm(error[:3, 3]) * 1000.0
                    rotation_deg = rotation_error_degrees(error[:3, :3])
                    validation = 'error: {:.1f} mm | {:.2f} deg'.format(
                        translation_mm, rotation_deg,
                    )
                    color = (40, 220, 40) if translation_mm <= 10.0 and rotation_deg <= 5.0 else (0, 80, 255)
                except Exception as exc:
                    validation = 'validation TF unavailable: {}'.format(exc)
                    color = (0, 80, 255)
                cv2.putText(output, validation, (12, 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        status = 'validation | end frame: {}'.format(self.args.end_frame)
        if self.args.mode == 'collect':
            status = 'samples: {} | end frame: {}'.format(
                len(self.samples), self.args.end_frame,
            )
        cv2.putText(output, status, (12, output.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 0), 1)
        cv2.putText(output, 'A/D W/S R/F T/G Y/H jog | Space stop | motion {}'.format(
            'ON' if self.motion_enabled else 'OFF',
        ), (12, output.shape[0] - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 220, 255) if self.motion_enabled else (160, 160, 160), 1)
        return output

    def handle_key(self, key):
        """Handle a Tk key while keeping motion commands explicit."""
        if key in ('q', 'Q', 'Escape'):
            self.stop_motion()
            return False
        if key == ' ' and self.motion_enabled:
            self.stop_motion()
            return True
        key = key.lower()
        if key in {'a', 'd', 'w', 's', 'r', 'f', 't', 'g', 'y', 'h'}:
            mapping = {
                'a': (6, -1, 'base left'), 'd': (6, 1, 'base right'),
                'w': (5, 1, 'shoulder up'), 's': (5, -1, 'shoulder down'),
                'r': (4, 1, 'elbow up'), 'f': (4, -1, 'elbow down'),
                't': (3, 1, 'wrist pitch +'), 'g': (3, -1, 'wrist pitch -'),
                'y': (2, 1, 'wrist roll +'), 'h': (2, -1, 'wrist roll -'),
            }
            servo_id, direction, label = mapping[key]
            self.jog_motion(servo_id, direction, label)
        return True

    def save_detection(self, detection, stamp):
        if detection is None or stamp is None:
            return
        try:
            base_to_end = self.robot_transform(stamp)
        except Exception as exc:
            rospy.logwarn('cannot save: TF %s -> %s unavailable: %s',
                          self.args.base_frame, self.args.end_frame, exc)
            return
        _, ids, camera_to_target = detection
        self.samples.append({
            'stamp': stamp.to_sec(),
            'charuco_corners': int(len(ids)),
            'base_to_end': matrix_to_dict(base_to_end),
            'camera_to_target': matrix_to_dict(camera_to_target),
        })
        self.save_samples()
        rospy.loginfo('saved calibration sample %d/%d',
                      len(self.samples), self.args.target_samples)

    def run(self):
        # Tk avoids the GTK/MIT-SHM path, which is unavailable in this container's
        # private IPC namespace while still allowing a normal X11 desktop window.
        root = tk.Tk()
        root.title('ArmPi Eye-to-Hand Calibration')
        root.geometry('960x720')
        root.configure(background='black')
        label = tk.Label(root, background='black')
        label.pack(fill=tk.BOTH, expand=True)
        root.focus_force()
        state = {
            'photo': None, 'detection': None, 'stamp': None,
            'closed': False, 'pressed': set(),
        }

        def close():
            if not state['closed']:
                state['closed'] = True
                self.stop_motion()
                root.destroy()

        def keypress(event):
            key = event.keysym if event.keysym == 'Escape' else event.char
            if event.keysym in state['pressed']:
                return
            state['pressed'].add(event.keysym)
            if key == 'S':
                if self.args.mode == 'collect':
                    self.save_detection(state['detection'], state['stamp'])
            elif not self.handle_key(key):
                close()

        def keyrelease(event):
            state['pressed'].discard(event.keysym)

        def update():
            if state['closed'] or rospy.is_shutdown():
                close()
                return
            with self.lock:
                image = None if self.latest_image is None else self.latest_image.copy()
                stamp = self.latest_stamp
                camera_matrix = self.camera_matrix
                distortion = self.distortion
            if image is None or camera_matrix is None or distortion is None:
                display = np.full((480, 640, 3), 25, dtype=np.uint8)
                cv2.putText(display, 'Waiting for camera image and CameraInfo...',
                            (35, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 180, 255), 2)
                detection = None
            else:
                detection = detect_board(
                    image, self.board, camera_matrix, distortion,
                    self.args.minimum_corners,
                )
                display = self.draw(image, detection, stamp)
            state['detection'] = detection
            state['stamp'] = stamp
            height, width = display.shape[:2]
            scale = min(960.0 / width, 680.0 / height, 1.0)
            if scale != 1.0:
                display = cv2.resize(display, (int(width * scale), int(height * scale)))
            # Tk can consume PNG bytes directly, avoiding Pillow's optional
            # ImageTk extension and preserving the RGB channel order.
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            encoded = cv2.imencode('.png', rgb)[1]
            photo = tk.PhotoImage(data=encoded.tobytes())
            state['photo'] = photo
            label.configure(image=photo)
            root.after(50, update)

        root.protocol('WM_DELETE_WINDOW', close)
        root.bind('<KeyPress>', keypress)
        root.bind('<KeyRelease>', keyrelease)
        update()
        root.mainloop()
        return len(self.samples)


def run_solver(args):
    with open(args.samples, 'r', encoding='utf-8') as stream:
        payload = json.load(stream)
    result = solve_samples(payload.get('samples', []))
    base_to_camera = result['base_to_camera']
    end_to_board = result['end_to_board']
    output = {
        'base_frame': payload.get('base_frame', args.base_frame),
        'camera_frame': args.camera_frame,
        'end_frame': payload.get('end_frame', args.end_frame),
        'board_frame': args.board_frame,
        'base_to_camera': matrix_to_dict(base_to_camera),
        'end_to_board': matrix_to_dict(end_to_board),
        'residuals': result['residuals'],
        'sample_count': len(payload.get('samples', [])),
    }
    directory = os.path.dirname(args.output)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2)
    translations = base_to_camera[:3, 3]
    print('wrote {}'.format(args.output))
    print('base -> camera translation (m): {:.4f} {:.4f} {:.4f}'.format(*translations))
    print('residual translation max (m): {:.4f}'.format(
        max(item['translation_m'] for item in result['residuals']),
    ))
    print('residual rotation max (deg): {:.2f}'.format(
        max(item['rotation_deg'] for item in result['residuals']),
    ))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('collect', 'solve', 'validate'), default='collect')
    parser.add_argument('--samples', default='/home/ubuntu/share/tmp/ArmPi-FPV/output/eye_to_hand_samples.json')
    parser.add_argument('--output', default='/home/ubuntu/share/tmp/ArmPi-FPV/output/eye_to_hand_result.json')
    parser.add_argument('--result', default='/home/ubuntu/share/tmp/ArmPi-FPV/output/eye_to_hand_result.json')
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/color/camera_info')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--end-frame', default='grasping_frame')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--board-frame', default='calibration_board')
    parser.add_argument('--minimum-corners', type=int, default=8)
    parser.add_argument('--target-samples', type=int, default=15)
    # roslaunch appends private remapping arguments such as __name and __log.
    # rospy.myargv removes those before argparse sees the application options.
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()
    if args.mode == 'solve':
        run_solver(args)
        return
    rospy.init_node('eye_to_hand_calibration_collector')
    # roslaunch supplies these values as private parameters.  Apply them after
    # init_node so collection uses the same frames and output path shown by the
    # launch configuration rather than argparse defaults.
    for name in (
        'image_topic', 'camera_info_topic', 'base_frame', 'end_frame',
        'samples', 'target_samples',
    ):
        setattr(args, name, rospy.get_param('~' + name, getattr(args, name)))
    count = Collector(args).run()
    rospy.loginfo('collector stopped with %d samples', count)


if __name__ == '__main__':
    main()
