#!/usr/bin/env python3
import threading

import rospy
from std_msgs.msg import Float64

from desk_cleaning.simulation import ARM_SERVO_IDS, arm_pulses_to_radians, gripper_pulse_to_radians, interpolate
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, ServoState, ServoStateList


class SimServoBridge:
    def __init__(self):
        self.lock = threading.Lock()
        home = rospy.get_param('~initial_pulses', [500, 625, 825, 80, 500])
        self.current = {servo_id: float(value) for servo_id, value in zip(ARM_SERVO_IDS, home)}
        self.current[1] = float(rospy.get_param('~initial_gripper_pulse', 200))
        self.start = dict(self.current)
        self.goal = dict(self.current)
        self.started_at = {servo_id: rospy.Time.now() for servo_id in self.current}
        self.duration = {servo_id: 0.0 for servo_id in self.current}
        self.publishers = {
            name: rospy.Publisher('/%s_position_controller/command' % name, Float64, queue_size=1)
            for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')
        }
        self.state_pub = rospy.Publisher(
            '/servo_controllers/port_id_1/servo_states', ServoStateList, queue_size=1,
        )
        rospy.Subscriber(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur,
            self.command_callback,
            queue_size=10,
        )
        rate = max(10.0, float(rospy.get_param('~rate', 50.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.update)
        rospy.loginfo('Gazebo servo bridge ready')

    def command_callback(self, msg):
        now = rospy.Time.now()
        with self.lock:
            for command in msg.id_pos_dur_list:
                servo_id = int(command.id)
                if servo_id not in self.current:
                    rospy.logwarn_throttle(2.0, 'ignoring unsupported servo id %d', servo_id)
                    continue
                self.start[servo_id] = self.current[servo_id]
                self.goal[servo_id] = min(1000.0, max(0.0, float(command.position)))
                self.started_at[servo_id] = now
                self.duration[servo_id] = max(0.0, float(command.duration))

    def update(self, _event):
        now = rospy.Time.now()
        with self.lock:
            for servo_id in self.current:
                elapsed = (now - self.started_at[servo_id]).to_sec()
                duration = self.duration[servo_id]
                progress = 1.0 if duration <= 0.0 else elapsed / duration
                self.current[servo_id] = interpolate(
                    self.start[servo_id], self.goal[servo_id], progress,
                )
            pulses = dict(self.current)
            goals = dict(self.goal)

        arm_angles = arm_pulses_to_radians([pulses[servo_id] for servo_id in ARM_SERVO_IDS])
        for name, angle in zip(('joint1', 'joint2', 'joint3', 'joint4', 'joint5'), arm_angles):
            self.publishers[name].publish(Float64(data=angle))
        self.publishers['r_joint'].publish(Float64(data=gripper_pulse_to_radians(pulses[1])))

        states = ServoStateList()
        timestamp = now.to_sec()
        for servo_id in sorted(pulses):
            state = ServoState()
            state.timestamp = timestamp
            state.id = servo_id
            state.goal = int(round(goals[servo_id]))
            state.position = int(round(pulses[servo_id]))
            state.error = state.goal - state.position
            state.voltage = 7400
            states.servo_states.append(state)
        self.state_pub.publish(states)


if __name__ == '__main__':
    rospy.init_node('sim_servo_bridge')
    SimServoBridge()
    rospy.spin()
