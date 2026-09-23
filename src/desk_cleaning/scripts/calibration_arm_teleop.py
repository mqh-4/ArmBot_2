#!/usr/bin/env python3
"""Conservative terminal jogger for eye-to-hand calibration.

Motion is disabled unless the ROS parameter ``~allow_motion`` is true.
Only arm servos 2-6 are addressed; the removed gripper servo is untouched.
"""

import select
import sys
import termios
import tty

import rospy

from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur
from hiwonder_servo_msgs.msg import ServoStateList


class CalibrationArmTeleop:
    # The mapping follows the ArmPi controller's servo IDs: 6 is the base and
    # 2 is the wrist rotation. Directions are intentionally only small jogs.
    KEY_TO_SERVO = {
        'a': (6, -1, 'base left'),
        'd': (6, 1, 'base right'),
        'w': (5, 1, 'shoulder up'),
        's': (5, -1, 'shoulder down'),
        'r': (4, 1, 'elbow up'),
        'f': (4, -1, 'elbow down'),
        't': (3, 1, 'wrist pitch +'),
        'g': (3, -1, 'wrist pitch -'),
        'y': (2, 1, 'wrist roll +'),
        'h': (2, -1, 'wrist roll -'),
    }

    def __init__(self):
        self.allow_motion = bool(rospy.get_param('~allow_motion', False))
        self.step = max(1, min(50, int(rospy.get_param('~step_pulse', 15))))
        self.duration = max(0.3, min(3.0, float(rospy.get_param('~duration', 1.0))))
        self.minimum = max(20, min(400, int(rospy.get_param('~minimum_pulse', 100))))
        self.maximum = min(980, max(600, int(rospy.get_param('~maximum_pulse', 900))))
        self.pulses = {}
        self.publisher = rospy.Publisher(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur,
            queue_size=1,
        )
        rospy.Subscriber(
            '/servo_controllers/port_id_1/servo_states',
            ServoStateList,
            self.state_callback,
            queue_size=1,
        )

    def state_callback(self, message):
        for state in message.servo_states:
            if int(state.id) in (2, 3, 4, 5, 6):
                self.pulses[int(state.id)] = int(state.position)

    def print_help(self):
        rospy.loginfo(
            '\nCalibration jog keys (one small step each):\n'
            '  a/d base left/right     w/s shoulder up/down\n'
            '  r/f elbow up/down       t/g wrist pitch +/-\n'
            '  y/h wrist roll +/-      space stop\n'
            '  q or Esc quit\n'
            '  allow_motion=%s, step=%d pulse, duration=%.1fs\n',
            self.allow_motion, self.step, self.duration,
        )

    def publish_positions(self, positions, duration):
        if not self.allow_motion:
            rospy.logwarn('motion disabled; would send %s', positions)
            return
        message = MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(servo_id), int(position), float(duration))
                for servo_id, position in sorted(positions.items())
            ],
        )
        self.publisher.publish(message)

    def stop(self):
        if not self.pulses:
            rospy.logwarn('no servo state received; cannot issue stop command')
            return
        self.publish_positions(dict(self.pulses), 0.15)

    def jog(self, servo_id, direction, label):
        if servo_id not in self.pulses:
            rospy.logwarn('no state for servo %d yet; wait for servo feedback', servo_id)
            return
        target = self.pulses[servo_id] + direction * self.step
        target = max(self.minimum, min(self.maximum, target))
        if target == self.pulses[servo_id]:
            rospy.logwarn('servo %d is at the configured safety limit', servo_id)
            return
        self.pulses[servo_id] = target
        rospy.loginfo('%s: servo %d -> %d', label, servo_id, target)
        self.publish_positions({servo_id: target}, self.duration)

    def run(self):
        self.print_help()
        if not sys.stdin.isatty():
            rospy.logerr('teleop requires an interactive terminal (docker exec -it)')
            return
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    key = sys.stdin.read(1)
                    if key in ('q', '\x1b'):
                        break
                    if key == ' ':
                        self.stop()
                    elif key in self.KEY_TO_SERVO:
                        servo_id, direction, label = self.KEY_TO_SERVO[key]
                        self.jog(servo_id, direction, label)
                rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            if self.allow_motion:
                self.stop()


if __name__ == '__main__':
    rospy.init_node('calibration_arm_teleop')
    CalibrationArmTeleop().run()
