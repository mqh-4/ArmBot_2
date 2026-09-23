#!/usr/bin/env python3
import math
import threading
import time

import actionlib
import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from armpi_fpv_kinematics.kinematics_control import (
    set_joint_value_target,
    set_pose_target,
)
from desk_cleaning.core import (
    Detection,
    Workspace,
    adaptive_gripper_open_pulse,
    base_to_ik_position,
    select_target,
)
from desk_cleaning.msg import (
    CleanDeskAction,
    CleanDeskFeedback,
    CleanDeskGoal,
    CleanDeskResult,
    DetectedObjectArray,
)
from desk_cleaning.srv import SetSimGrasp
from hiwonder_servo_controllers import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur


class MotionCancelled(Exception):
    pass


class DeskCleaningExecutor:
    def __init__(self):
        self.lock = threading.RLock()
        self.latest_detections = []
        self.latest_detection_time = rospy.Time(0)
        # Camera drivers can publish hardware timestamps that are not on the
        # same clock as the ROS master. Keep a local receipt time for safety
        # freshness checks instead of rejecting a live detection as stale.
        self.latest_detection_received_time = rospy.Time(0)
        self.perception_ready = False
        self.track_counts = {}
        self.track_positions = {}
        # Regions that have already received an object during this job. These
        # persist even if the red tape is briefly occluded.
        self.placed_box_regions = []
        self.completed_total = 0
        self.motion = rospy.get_param('~motion')
        self.safety = rospy.get_param('~safety')
        bounds = rospy.get_param('~workspace')
        self.workspace = Workspace(**bounds)
        self.destinations = rospy.get_param('~destinations')
        object_config = rospy.get_param('~objects', {})
        # The object configuration and detection topic use separate private
        # names so a topic remap cannot hide the configured class allowlist.
        if isinstance(object_config, dict) and isinstance(object_config.get('objects'), dict):
            object_config = object_config['objects']
        self.enabled_classes = {
            name for name, config in object_config.items() if config.get('enabled', False)
        }
        # The box marker is a placement reference, not an object to grasp.
        # Keep its latest position separately so spoken "放入盒子" commands
        # can use the measured center without adding it to the object allowlist.
        self.box_marker_class = str(self.destinations.get(
            'apple_box', {},
        ).get('marker_class', 'apple_box'))
        self.box_boundary_margin_px = int(self.destinations.get(
            'apple_box', {},
        ).get('boundary_margin_px', 4))
        self.max_attempts = int(self.safety.get('max_attempts_per_object', 2))
        self.max_age = float(self.safety.get('detection_max_age', 1.5))
        self.max_age_by_class = {
            str(name): float(value)
            for name, value in self.safety.get('detection_max_age_by_class', {}).items()
        }
        self.max_grasp_width = float(self.safety.get('max_grasp_width', 0.075))
        # Banana detections are intentionally elongated: the gripper closes
        # across the short side, while the long side is checked separately by
        # max_object_extent_by_class. Keep these class defaults in code as a
        # guard against an older workspace.yaml being left on the robot.
        configured_width_limits = {
            str(name): float(value)
            for name, value in self.safety.get('max_grasp_width_by_class', {}).items()
        }
        self.max_grasp_width_by_class = dict(configured_width_limits)
        self.max_grasp_width_by_class['banana'] = max(
            0.18,
            self.max_grasp_width_by_class.get('banana', 0.18),
        )
        # Oranges are round and their RGB-D bbox can vary by a few millimetres
        # as the detector edge changes. Keep a dedicated width allowance
        # instead of applying the tighter generic 75 mm limit.
        self.max_grasp_width_by_class['orange'] = max(
            0.09,
            self.max_grasp_width_by_class.get('orange', 0.09),
        )
        self.max_object_extent = float(self.safety.get('max_object_extent', 0.20))
        configured_extent_limits = {
            str(name): float(value)
            for name, value in self.safety.get('max_object_extent_by_class', {}).items()
        }
        self.max_object_extent_by_class = dict(configured_extent_limits)
        self.max_object_extent_by_class['banana'] = max(
            0.30,
            self.max_object_extent_by_class.get('banana', 0.30),
        )
        self.min_confidence_by_class = {
            str(name): float(value)
            for name, value in self.safety.get('min_confidence_by_class', {}).items()
        }
        # Preserve valid-depth and workspace checks while tolerating the
        # observed orange confidence range around 0.39.
        self.min_confidence_by_class['orange'] = min(
            0.35,
            self.min_confidence_by_class.get('orange', 0.35),
        )
        self.required_frames = int(self.safety.get('required_detection_frames', 3))
        self.required_frames_by_class = {
            str(name): int(value)
            for name, value in self.safety.get('required_detection_frames_by_class', {}).items()
        }
        self.position_tolerance = float(self.safety.get('position_tolerance', 0.015))
        self.dry_run = bool(self.safety.get('dry_run', True))
        perception = rospy.get_param('~perception', {})
        self.eye_in_hand = bool(perception.get('eye_in_hand', True))
        self.observation_settle_time = float(
            perception.get('observation_settle_time', 1.0),
        )
        verification = rospy.get_param('~verification', {})
        self.verification_enabled = bool(verification.get('enabled', True)) and not self.dry_run
        self.verification_timeout = float(verification.get('timeout', 3.0))
        self.verification_required_frames = int(verification.get('required_frames', 3))
        self.verification_tolerance = float(verification.get('position_tolerance', 0.035))
        self.simulation_enabled = bool(rospy.get_param('~simulation/enabled', False))
        self.sim_grasp_service = rospy.get_param(
            '~simulation/grasp_service', '/desk_cleaning/sim_grasp',
        )
        self.sim_grasp = (
            rospy.ServiceProxy(self.sim_grasp_service, SetSimGrasp)
            if self.simulation_enabled else None
        )

        self.servo_pub = rospy.Publisher(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur,
            queue_size=1,
        )
        self.speech_pub = rospy.Publisher('/desk_cleaning/speech', String, queue_size=10)
        rospy.Subscriber('~detections', DetectedObjectArray, self.detection_callback, queue_size=1)
        rospy.Subscriber('~perception_ready', Bool, self.ready_callback, queue_size=1)

        self.server = actionlib.SimpleActionServer(
            '/desk_cleaning/clean_desk', CleanDeskAction, self.execute, auto_start=False,
        )
        self.server.start()
        self.local_client = actionlib.SimpleActionClient('/desk_cleaning/clean_desk', CleanDeskAction)
        rospy.Service('/desk_cleaning/start', Trigger, self.start_service)
        rospy.Service('/desk_cleaning/stop', Trigger, self.stop_service)
        rospy.loginfo(
            'desk cleaning executor ready (dry_run=%s, simulation=%s)',
            self.dry_run,
            self.simulation_enabled,
        )
        rospy.loginfo(
            'grasp size limits: default_width=%.3fm orange_width=%.3fm '
            'banana_width=%.3fm banana_extent=%.3fm',
            self.max_grasp_width,
            self.max_grasp_width_by_class.get('orange', self.max_grasp_width),
            self.max_grasp_width_by_class.get('banana', self.max_grasp_width),
            self.max_object_extent_by_class.get('banana', self.max_object_extent),
        )

    def detection_callback(self, msg):
        detections = []
        for item in msg.objects:
            # A general pretrained detector can publish many COCO classes.
            # Only explicitly enabled objects are ever eligible for grasping.
            if item.class_name not in self.enabled_classes and item.class_name != self.box_marker_class:
                if item.class_name in ('apple', 'orange', 'banana'):
                    rospy.logwarn_throttle(
                        3.0,
                        'fruit class=%s rejected: not enabled in executor objects config',
                        item.class_name,
                    )
                continue
            if item.class_name == self.box_marker_class:
                detections.append(Detection(
                    object_id=item.id,
                    class_name=item.class_name,
                    display_name=item.display_name,
                    confidence=item.confidence,
                    position=(item.position.x, item.position.y, item.position.z),
                    bbox=(item.x_min, item.y_min, item.x_max, item.y_max),
                    size=(item.size_x, item.size_y, item.size_z),
                ))
                continue
            min_confidence = self.min_confidence_by_class.get(
                item.class_name, float(self.safety.get('min_confidence', 0.55)),
            )
            if not item.valid_depth or item.confidence < min_confidence:
                if item.class_name in ('apple', 'orange', 'banana'):
                    rospy.logwarn_throttle(
                        3.0,
                        'fruit class=%s rejected: valid_depth=%s confidence=%.3f required=%.3f',
                        item.class_name, item.valid_depth, item.confidence, min_confidence,
                    )
                continue
            dimensions = [value for value in (item.size_x, item.size_y) if value > 0]
            max_width = self.max_grasp_width_by_class.get(
                item.class_name, self.max_grasp_width,
            )
            max_extent = self.max_object_extent_by_class.get(
                item.class_name, self.max_object_extent,
            )
            if dimensions and (
                min(dimensions) > max_width
                or max(dimensions) > max_extent
            ):
                if item.class_name in ('apple', 'orange', 'banana'):
                    rospy.logwarn_throttle(
                        3.0,
                        'fruit class=%s rejected: dimensions=(%.3f,%.3f) '
                        'limits=(width %.3f, extent %.3f)',
                        item.class_name, item.size_x, item.size_y,
                        max_width, max_extent,
                    )
                continue
            detections.append(Detection(
                object_id=item.id,
                class_name=item.class_name,
                display_name=item.display_name,
                confidence=item.confidence,
                position=(item.position.x, item.position.y, item.position.z),
                bbox=(item.x_min, item.y_min, item.x_max, item.y_max),
                grasp_yaw=item.grasp_yaw,
                size=(item.size_x, item.size_y, item.size_z),
            ))
        with self.lock:
            # A throttled detector can publish an empty frame between valid
            # inferences. Keep the last valid track until detection_max_age
            # expires; this removes visual/inference flicker without making
            # an object valid indefinitely.
            if not detections:
                return
            visible_ids = set()
            for item in detections:
                visible_ids.add(item.object_id)
                previous = self.track_positions.get(item.object_id)
                if previous is None:
                    self.track_counts[item.object_id] = 1
                else:
                    distance = math.sqrt(sum(
                        (current - old) ** 2
                        for current, old in zip(item.position, previous)
                    ))
                    self.track_counts[item.object_id] = (
                        self.track_counts.get(item.object_id, 0) + 1
                        if distance <= self.position_tolerance else 1
                    )
                self.track_positions[item.object_id] = item.position
            for missing_id in set(self.track_counts) - visible_ids:
                self.track_counts.pop(missing_id, None)
                self.track_positions.pop(missing_id, None)
            self.latest_detections = detections
            self.latest_detection_received_time = rospy.Time.now()
            self.latest_detection_time = (
                msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
            )
            apple_items = [item for item in detections if item.class_name == 'apple']
            if apple_items:
                rospy.loginfo_throttle(
                    2.0,
                    'apple track id=%s count=%s position=%s age_stamp=%.3f',
                    apple_items[0].object_id,
                    self.track_counts.get(apple_items[0].object_id, 0),
                    apple_items[0].position,
                    self.latest_detection_time.to_sec(),
                )

    def ready_callback(self, msg):
        self.perception_ready = msg.data

    def reset_detection_history(self):
        with self.lock:
            self.latest_detections = []
            self.latest_detection_time = rospy.Time(0)
            self.latest_detection_received_time = rospy.Time(0)
            self.track_counts.clear()
            self.track_positions.clear()

    def start_service(self, _request):
        goal = CleanDeskGoal()
        goal.destination = 'default_tray'
        goal.announce_each = True
        self.local_client.send_goal(goal)
        return TriggerResponse(success=True, message='desk cleanup started')

    def stop_service(self, _request):
        self.local_client.cancel_all_goals()
        return TriggerResponse(success=True, message='desk cleanup stop requested')

    def speak(self, text):
        self.speech_pub.publish(String(data=text))

    def feedback(self, state, current='', completed=0, remaining=0, message=''):
        msg = CleanDeskFeedback()
        msg.state = state
        msg.current_object = current
        msg.completed_count = completed
        msg.remaining_count = remaining
        msg.message = message
        self.server.publish_feedback(msg)

    def check_cancelled(self):
        if self.server.is_preempt_requested() or rospy.is_shutdown():
            raise MotionCancelled()

    def sleep(self, duration):
        deadline = time.monotonic() + max(0.0, float(duration))
        while time.monotonic() < deadline:
            self.check_cancelled()
            rospy.sleep(min(0.05, deadline - time.monotonic()))

    def current_detections(self):
        with self.lock:
            age = (rospy.Time.now() - self.latest_detection_received_time).to_sec()
            if self.latest_detections:
                for item in self.latest_detections:
                    class_age = self.max_age_by_class.get(item.class_name, self.max_age)
                    required = self.required_frames_by_class.get(
                        item.class_name, self.required_frames,
                    )
                    rospy.loginfo_throttle(
                        2.0,
                        'candidate class=%s id=%s confidence=%.3f age=%.3f max_age=%.3f '
                        'track=%s required=%s in_workspace=%s position=%s',
                        item.class_name,
                        item.object_id,
                        item.confidence,
                        age,
                        class_age,
                        self.track_counts.get(item.object_id, 0),
                        required,
                        self.workspace.contains(item.position),
                        item.position,
                    )
            current = [
                item for item in self.latest_detections
                if age <= self.max_age_by_class.get(item.class_name, self.max_age)
                if self.track_counts.get(item.object_id, 0) >= self.required_frames_by_class.get(
                    item.class_name, self.required_frames,
                )
            ]
            if current:
                rospy.loginfo_throttle(
                    2.0,
                    'current detections=%s age=%.3f tracks=%s',
                    [(item.class_name, item.object_id, item.position) for item in current],
                    age,
                    dict(self.track_counts),
                )
            return current

    def current_box_destination(self):
        """Return a fresh marker-centre destination, or configured fallback."""
        fallback = self.destinations.get('apple_box', {})
        with self.lock:
            age = (rospy.Time.now() - self.latest_detection_received_time).to_sec()
            markers = [item for item in self.latest_detections
                       if item.class_name == self.box_marker_class]
        if markers and age <= self.max_age:
            item = markers[0]
            box_height = float(fallback.get('box_height', 0.03))
            release_clearance = float(fallback.get('release_clearance', 0.045))
            # A one-edge boundary is sufficient to block re-grasping inside
            # the wall-side region, but its image bbox is not a box-centre
            # measurement. Keep the recorded centre for placement in that
            # case; a complete outline still uses its live RGB-D centre.
            if float(item.confidence) < 0.99:
                configured = dict(fallback)
                configured['z'] = float(configured.get('z', 0.04)) + box_height + release_clearance
                rospy.loginfo_throttle(
                    2.0,
                    'partial red boundary is exclusion-only; using configured apple_box centre '
                    '(%.3f, %.3f, %.3f)',
                    float(configured.get('x', 0.0)),
                    float(configured.get('y', 0.0)),
                    float(configured.get('z', 0.0)),
                )
                return configured
            return {
                'x': float(item.position[0]),
                'y': float(item.position[1]),
                # Marker is on the box centre/rim plane; release inside the
                # 3 cm box while retaining clearance from its bottom.
                'z': float(item.position[2]) + box_height + release_clearance,
                'yaw': 0.0,
            }
        if bool(fallback.get('require_boundary', True)):
            raise RuntimeError('red box boundary was not detected; refusing apple_box placement')
        fallback = dict(fallback)
        fallback['z'] = float(fallback.get('z', 0.04)) + float(
            fallback.get('box_height', 0.03),
        ) + float(fallback.get('release_clearance', 0.045))
        return fallback

    def is_inside_box_boundary(self, item):
        """Ignore detections whose image centre lies inside the red tape."""
        if item.class_name == self.box_marker_class:
            return True
        center_x = 0.5 * (item.bbox[0] + item.bbox[2])
        center_y = 0.5 * (item.bbox[1] + item.bbox[3])
        with self.lock:
            boundaries = [marker for marker in self.latest_detections
                          if marker.class_name == self.box_marker_class
                          and marker.bbox[2] > marker.bbox[0]
                          and marker.bbox[3] > marker.bbox[1]]
            placed_regions = list(self.placed_box_regions)
        margin = self.box_boundary_margin_px
        image_inside = any(
            marker.bbox[0] + margin <= center_x <= marker.bbox[2] - margin
            and marker.bbox[1] + margin <= center_y <= marker.bbox[3] - margin
            for marker in boundaries
        )
        if image_inside:
            return True
        # Once an object has been released, suppress anything detected in the
        # recorded 3-D box region, even when the wall-side tape is not visible.
        return any(
            abs(float(item.position[0]) - region['x']) <= region['half_x']
            and abs(float(item.position[1]) - region['y']) <= region['half_y']
            for region in placed_regions
        )

    def box_exclusion_region(self, destination):
        """Build a persistent 3-D no-regrasp region around the box centre."""
        configured = self.destinations.get('apple_box', {})
        margin = float(configured.get('exclusion_margin', 0.0))
        return {
            'x': float(destination.get('x', configured.get('x', 0.0))),
            'y': float(destination.get('y', configured.get('y', 0.0))),
            'half_x': float(configured.get('exclusion_half_extent_x', 0.12)) + margin,
            'half_y': float(configured.get('exclusion_half_extent_y', 0.12)) + margin,
        }

    def remember_placed_box_region(self, destination):
        region = self.box_exclusion_region(destination)
        with self.lock:
            if not any(
                abs(region['x'] - old['x']) < 0.01
                and abs(region['y'] - old['y']) < 0.01
                for old in self.placed_box_regions
            ):
                self.placed_box_regions.append(region)
        rospy.loginfo(
            'placed-object exclusion region center=(%.3f, %.3f) half=(%.3f, %.3f)',
            region['x'], region['y'], region['half_x'], region['half_y'],
        )

    def wait_for_box_boundary(self):
        """Wait for a fresh, multi-frame boundary before selecting an apple."""
        timeout = float(self.safety.get(
            'box_boundary_wait_timeout', self.safety.get('scan_timeout', 8.0),
        ))
        deadline = time.monotonic() + max(0.1, timeout)
        while time.monotonic() < deadline:
            self.check_cancelled()
            markers = [item for item in self.current_detections()
                       if item.class_name == self.box_marker_class]
            if markers:
                rospy.loginfo_throttle(
                    2.0,
                    'red box boundary stable before target selection (id=%s confidence=%.2f)',
                    markers[0].object_id, markers[0].confidence,
                )
                return
            self.feedback('WAITING_FOR_BOX', message='等待红色盒子边界稳定')
            self.sleep(0.2)
        raise RuntimeError('red box boundary was not detected; refusing apple_box cleanup')

    def verify_object_removed(self, target):
        """Confirm a released object is absent from its original 3-D position."""
        if not self.verification_enabled:
            return
        self.feedback('VERIFYING', target.display_name, message='检查抓取结果')
        self.reset_detection_history()
        deadline = time.monotonic() + self.verification_timeout
        while time.monotonic() < deadline:
            self.check_cancelled()
            with self.lock:
                age = (rospy.Time.now() - self.latest_detection_time).to_sec()
                fresh = self.latest_detection_time != rospy.Time(0) and age <= self.max_age
                detections = list(self.latest_detections)
                stable_counts = dict(self.track_counts)
            if fresh:
                same_place = [
                    item for item in detections
                    if item.class_name == target.class_name
                    and stable_counts.get(item.object_id, 0) >= self.verification_required_frames
                    and math.sqrt(sum(
                        (current - old) ** 2
                        for current, old in zip(item.position, target.position)
                    )) <= self.verification_tolerance
                ]
                if same_place:
                    raise RuntimeError('%s仍在原位置，视觉复检失败' % target.display_name)
                return
            self.sleep(0.1)
        raise RuntimeError('视觉复检未获得新鲜且稳定的 RGB-D 检测')

    def solve_pose(self, position, roll_degrees):
        # Detection and workspace coordinates are base_link/TF coordinates;
        # the legacy IK library uses its own XY frame.
        ik_position = base_to_ik_position(position)
        if self.dry_run:
            return [500, 500, 500, 500, 500]
        response = set_pose_target(
            ik_position,
            float(self.motion.get('pitch', -175.0)),
            [
                float(self.motion.get('pitch_min', -180.0)),
                float(self.motion.get('pitch_max', -120.0)),
            ],
            roll_degrees,
        )
        if not response[1]:
            raise RuntimeError('inverse kinematics has no solution for %s' % (position,))
        pulse = [int(round(value)) for value in response[1]]
        # The legacy service has an XY frame rotated relative to base_link.
        # Verify its own FK result before moving hardware so a bad branch or
        # a future coordinate conversion regression cannot silently move the
        # gripper away from the RGB-D target.
        fk = set_joint_value_target([float(value) for value in pulse])
        if not fk.solution:
            raise RuntimeError('inverse kinematics FK check failed for %s' % (position,))
        actual = (-fk.pose.position.y, fk.pose.position.x, fk.pose.position.z)
        error = math.sqrt(sum((wanted - measured) ** 2
                              for wanted, measured in zip(position, actual)))
        tolerance = float(self.motion.get('ik_fk_tolerance', 0.003))
        rospy.loginfo(
            'IK verified target=%s actual=%s error=%.4fm pulse=%s rpy=%s',
            position, actual, error, pulse, response[3],
        )
        if error > tolerance:
            raise RuntimeError(
                'inverse kinematics FK error %.4fm exceeds %.4fm for %s'
                % (error, tolerance, position),
            )
        return pulse

    def move_arm(self, pulse, duration):
        self.check_cancelled()
        commands = (
            (2, pulse[4]), (3, pulse[3]), (4, pulse[2]),
            (5, pulse[1]), (6, pulse[0]),
        )
        if self.dry_run:
            rospy.loginfo('dry-run arm command: %s', commands)
            self.sleep(0.05)
            return
        bus_servo_control.set_servos(self.servo_pub, duration, commands)
        self.sleep(duration + 0.1)

    def move_gripper(self, pulse, duration):
        self.check_cancelled()
        if self.dry_run:
            rospy.loginfo('dry-run gripper command: %s', pulse)
            self.sleep(0.05)
            return
        bus_servo_control.set_servos(self.servo_pub, duration, ((1, int(pulse)),))
        self.sleep(duration + 0.1)

    def set_simulated_grasp(self, object_id, attach):
        if not self.simulation_enabled:
            return
        try:
            rospy.wait_for_service(self.sim_grasp_service, timeout=3.0)
            response = self.sim_grasp(object_id=object_id, attach=attach)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise RuntimeError('Gazebo grasp service failed: %s' % exc)
        if not response.success:
            raise RuntimeError(response.message)

    def execute_pick_and_place(self, target, destination_name):
        x, y, z = target.position
        class_grasp_offsets = self.motion.get('grasp_z_offset_by_class', {})
        grasp_offset = float(class_grasp_offsets.get(
            target.class_name, self.motion.get('grasp_z_offset', 0.012),
        ))
        grasp_z = z + grasp_offset
        approach_z = grasp_z + float(self.motion.get('approach_height', 0.055))
        lift_heights = self.motion.get('lift_height_by_class', {})
        lift_height = float(lift_heights.get(
            target.class_name, self.motion.get('lift_height', 0.080),
        ))
        lift_z = grasp_z + lift_height
        rospy.loginfo(
            'grasp target=%s base=(%.3f, %.3f, %.3f) offset_z=%.3f grasp_z=%.3f '
            'lift_height=%.3f',
            target.class_name, x, y, z, grasp_offset, grasp_z, lift_height,
        )
        destination = (
            self.current_box_destination()
            if destination_name == 'apple_box'
            else self.destinations[destination_name]
        )
        if destination_name == 'apple_box':
            # Box destination already contains its desired release height.
            drop_z = float(destination['z'])
        else:
            drop_z = float(destination['z']) + float(self.motion.get('drop_height', 0.055))
        release_offsets = self.motion.get('release_height_offset_by_class', {})
        release_offset = (
            float(release_offsets.get(target.class_name, 0.0))
            if destination_name == 'apple_box' else 0.0
        )
        drop_z += release_offset
        # Keep the raised banana release target inside the configured vertical
        # workspace even when RGB-D box depth fluctuates between frames.
        release_z_limit = float(self.workspace.z_max) - 0.005
        if drop_z > release_z_limit:
            rospy.logwarn(
                'release height %.3fm exceeds workspace limit %.3fm; clamping',
                drop_z, release_z_limit,
            )
            drop_z = release_z_limit
        rospy.loginfo(
            'placement target=%s base=(%.3f, %.3f, %.3f) release_z=%.3f '
            'class_offset_z=%.3f',
            destination_name,
            float(destination['x']), float(destination['y']),
            float(destination.get('z', 0.0)), drop_z, release_offset,
        )
        # A round apple has no meaningful in-plane grasp orientation. Using
        # its noisy detector box angle can make IK choose a reversed wrist
        # branch, so allow class-specific roll overrides in the motion config.
        roll_by_class = self.motion.get('roll_by_class', {})
        roll_offsets = self.motion.get('roll_offset_by_class', {})
        if target.class_name in roll_by_class:
            roll = float(roll_by_class[target.class_name])
        else:
            roll = math.degrees(target.grasp_yaw) + float(
                roll_offsets.get(target.class_name, 0.0),
            )
        rospy.loginfo(
            'grasp orientation class=%s axis_deg=%.1f roll_deg=%.1f',
            target.class_name, math.degrees(target.grasp_yaw), roll,
        )
        gripper_profiles = self.motion.get('gripper_profiles', {})
        gripper_profile = gripper_profiles.get(target.class_name, {})
        base_open_pulse = int(gripper_profile.get(
            'open_pulse', self.motion.get('open_pulse', 200),
        ))
        close_pulse = int(gripper_profile.get(
            'close_pulse', self.motion.get('close_pulse', 560),
        ))
        if bool(gripper_profile.get('adaptive_opening', False)):
            gripper_size = target.size[:2]
            if str(gripper_profile.get('width_dimension', 'max')).lower() == 'min':
                valid_sizes = [value for value in gripper_size if value > 0]
                if valid_sizes:
                    narrow_width = min(valid_sizes)
                    gripper_size = (narrow_width, narrow_width)
            open_pulse = adaptive_gripper_open_pulse(
                gripper_size,
                base_pulse=base_open_pulse,
                reference_width=float(gripper_profile.get('reference_width', 0.07)),
                width_margin=float(gripper_profile.get(
                    'width_margin', self.motion.get('gripper_width_margin', 0.018),
                )),
                pulse_per_meter=float(gripper_profile.get(
                    'pulse_per_meter', self.motion.get('gripper_pulse_per_meter', 2000.0),
                )),
                min_pulse=float(gripper_profile.get(
                    'min_open_pulse', self.motion.get('min_open_pulse', 40),
                )),
                max_pulse=float(gripper_profile.get(
                    'max_open_pulse', self.motion.get('max_open_pulse', 220),
                )),
                extra_open_pulse=float(gripper_profile.get('extra_open_pulse', 0.0)),
            )
            measured_width = max((value for value in gripper_size if value > 0), default=0.0)
            rospy.loginfo(
                'adaptive gripper class=%s measured_width=%.3fm open_pulse=%d '
                '(base=%d margin=%.3fm)',
                target.class_name,
                measured_width,
                open_pulse,
                base_open_pulse,
                float(gripper_profile.get(
                    'width_margin', self.motion.get('gripper_width_margin', 0.018),
                )),
            )
        else:
            open_pulse = base_open_pulse
        move_duration = float(self.motion.get('move_duration', 1.2))
        gripper_duration = float(self.motion.get('gripper_duration', 0.5))

        # Solve every pose before moving so an unreachable later waypoint cannot strand the arm.
        approach = self.solve_pose((x, y, approach_z), roll)
        grasp = self.solve_pose((x, y, grasp_z), roll)
        lift = self.solve_pose((x, y, lift_z), roll)
        preserve_grasp_roll = set(self.motion.get('preserve_grasp_roll_by_class', []))
        placement_roll = (
            roll if target.class_name in preserve_grasp_roll
            else float(destination.get('yaw', 0.0))
        )
        rospy.loginfo(
            'placement orientation class=%s roll_deg=%.1f preserve_grasp_roll=%s',
            target.class_name, placement_roll, target.class_name in preserve_grasp_roll,
        )
        bin_approach = self.solve_pose(
            (float(destination['x']), float(destination['y']), drop_z),
            placement_roll,
        )

        attached = False
        try:
            self.move_gripper(open_pulse, gripper_duration)
            self.move_arm(approach, move_duration)
            self.move_arm(grasp, move_duration * 0.7)
            self.move_gripper(close_pulse, gripper_duration)
            self.set_simulated_grasp(target.object_id, True)
            attached = self.simulation_enabled
            self.move_arm(lift, move_duration * 0.7)
            self.move_arm(bin_approach, move_duration)
            self.move_gripper(open_pulse, gripper_duration)
            self.set_simulated_grasp(target.object_id, False)
            attached = False
            if destination_name == 'apple_box':
                self.remember_placed_box_region(destination)
            home = [int(value) for value in self.motion.get(
                'home_pulses', [500, 625, 825, 80, 500],
            )]
            self.move_arm(home, move_duration)
            if self.eye_in_hand:
                self.reset_detection_history()
                self.sleep(self.observation_settle_time)
            self.verify_object_removed(target)
        finally:
            if attached:
                try:
                    self.set_simulated_grasp(target.object_id, False)
                except RuntimeError as exc:
                    rospy.logwarn('failed to release simulated object: %s', exc)

    def execute(self, goal):
        result = CleanDeskResult()
        completed = []
        failed = []
        failed_ids = set()
        processed_ids = set()
        attempts = {}
        destination = goal.destination or 'default_tray'
        with self.lock:
            self.placed_box_regions = []
        if destination not in self.destinations:
            result.message = 'unknown destination: ' + destination
            self.server.set_aborted(result, result.message)
            return
        configured_object_config = rospy.get_param('~objects', {})
        if (isinstance(configured_object_config, dict)
                and isinstance(configured_object_config.get('objects'), dict)):
            configured_object_config = configured_object_config['objects']
        configured_classes = set(configured_object_config)
        unknown_classes = (set(goal.allowed_classes) | set(goal.excluded_classes)) - configured_classes
        if unknown_classes:
            result.message = 'unknown object classes: ' + ', '.join(sorted(unknown_classes))
            self.server.set_aborted(result, result.message)
            return

        forbidden_requested = set(goal.allowed_classes) - self.enabled_classes
        if forbidden_requested:
            result.message = 'requested object classes are disabled for safety'
            self.server.set_aborted(result, result.message)
            return
        allowed = list(goal.allowed_classes)
        excluded = (
            set(goal.excluded_classes)
            | (configured_classes - self.enabled_classes)
            | {self.box_marker_class}
        )
        limit = min(int(goal.max_objects), 100)
        try:
            self.speak('开始清理桌面')
            if self.eye_in_hand:
                self.feedback('POSITIONING_CAMERA', message='移动到观察姿态')
                home = [int(value) for value in self.motion.get(
                    'home_pulses', [500, 625, 825, 80, 500],
                )]
                self.move_arm(home, float(self.motion.get('move_duration', 1.2)))
                self.reset_detection_history()
                self.sleep(self.observation_settle_time)
            ready_deadline = time.monotonic() + float(self.safety.get('scan_timeout', 8.0))
            while not self.perception_ready and time.monotonic() < ready_deadline:
                self.check_cancelled()
                self.feedback('WAITING_FOR_CAMERA', message='等待深度相机和检测模型')
                self.sleep(0.1)
            if not self.perception_ready:
                result.message = '深度相机或目标检测模型未就绪'
                self.speak(result.message)
                self.server.set_aborted(result, result.message)
                return

            empty_frames = 0
            scan_started = time.monotonic()
            while not rospy.is_shutdown():
                self.check_cancelled()
                if destination == 'apple_box':
                    try:
                        # The box marker can arrive a few frames after the
                        # apple. Do not select an apple until the exclusion
                        # region and placement reference are both stable.
                        self.wait_for_box_boundary()
                    except MotionCancelled:
                        raise
                    except RuntimeError as exc:
                        result.message = str(exc)
                        self.speak(result.message)
                        self.server.set_aborted(result, result.message)
                        return
                snapshot = self.current_detections()
                available = [
                    item for item in snapshot
                    if item.object_id not in processed_ids
                    and not self.is_inside_box_boundary(item)
                ]
                target = select_target(
                    available, self.workspace, allowed, list(excluded), failed_ids,
                )
                self.feedback('SCANNING', completed=len(completed), remaining=len(available))
                if target is None:
                    empty_frames += 1
                    minimum_scan_elapsed = (
                        time.monotonic() - scan_started
                        >= float(self.safety.get('minimum_scan_duration', 2.0))
                    )
                    if minimum_scan_elapsed and empty_frames >= int(
                        self.safety.get('empty_confirmation_frames', 5)
                    ):
                        break
                    self.sleep(0.2)
                    continue
                empty_frames = 0
                if limit and len(completed) >= limit:
                    break

                self.feedback(
                    'GRASPING', target.display_name, len(completed), max(0, len(available) - 1),
                    '正在清理' + target.display_name,
                )
                try:
                    self.execute_pick_and_place(target, destination)
                    processed_ids.add(target.object_id)
                    completed.append(target.display_name)
                    self.completed_total += 1
                    if goal.announce_each:
                        self.speak(
                            '%s规划验证完成' % target.display_name
                            if self.dry_run else '%s已完成清理' % target.display_name
                        )
                    self.feedback('ANNOUNCE', target.display_name, len(completed), 0)
                    self.sleep(float(self.safety.get('settle_time', 0.8)))
                except MotionCancelled:
                    raise
                except Exception as exc:
                    attempts[target.object_id] = attempts.get(target.object_id, 0) + 1
                    rospy.logwarn('failed to clean %s: %s', target.display_name, exc)
                    if attempts[target.object_id] >= self.max_attempts:
                        failed_ids.add(target.object_id)
                        failed.append(target.display_name)
                        self.speak('%s暂时无法清理，已跳过' % target.display_name)

            result.success = bool(completed) or not failed
            result.completed_count = len(completed)
            result.completed_objects = completed
            result.failed_objects = failed
            result.message = (
                '桌面清理规划验证完成，共检查%d件物品' % len(completed)
                if self.dry_run
                else '桌面清理完成，共清理%d件物品' % len(completed)
            )
            if failed:
                result.message += '，跳过%d件' % len(failed)
            self.speak(result.message)
            self.feedback('DONE', completed=len(completed), message=result.message)
            self.server.set_succeeded(result, result.message)
        except MotionCancelled:
            result.success = False
            result.completed_count = len(completed)
            result.completed_objects = completed
            result.failed_objects = failed
            result.message = '桌面清理已停止'
            self.speak(result.message)
            self.server.set_preempted(result, result.message)


if __name__ == '__main__':
    rospy.init_node('desk_cleaning_executor')
    DeskCleaningExecutor()
    rospy.spin()
