#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class SimBaseAnchor:
    def __init__(self):
        self.model_name = rospy.get_param('~robot_model', 'robot')
        values = rospy.get_param('~robot_pose', [0.0, 0.0, 0.002, 0.0, 0.0, 0.0])
        self.state = ModelState()
        self.state.model_name = self.model_name
        self.state.reference_frame = 'world'
        self.state.pose.position.x, self.state.pose.position.y, self.state.pose.position.z = values[:3]
        roll, pitch, yaw = (float(value) for value in values[3:])
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        self.state.pose.orientation.w = cr * cp * cy + sr * sp * sy
        self.state.pose.orientation.x = sr * cp * cy - cr * sp * sy
        self.state.pose.orientation.y = cr * sp * cy + sr * cp * sy
        self.state.pose.orientation.z = cr * cp * sy - sr * sp * cy
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rate = max(5.0, float(rospy.get_param('~anchor_rate', 20.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.anchor)
        rospy.loginfo('anchoring Gazebo model %s to the desk', self.model_name)

    def anchor(self, _event):
        try:
            response = self.set_model_state(self.state)
            if not response.success:
                rospy.logwarn_throttle(2.0, 'failed to anchor robot: %s', response.status_message)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, 'robot anchor service failed: %s', exc)


if __name__ == '__main__':
    rospy.init_node('sim_base_anchor')
    SimBaseAnchor()
    rospy.spin()
