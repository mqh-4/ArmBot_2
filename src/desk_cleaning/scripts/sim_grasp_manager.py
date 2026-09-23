#!/usr/bin/env python3
import copy
import math
import threading

import rospy
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from gazebo_msgs.srv import SetModelState

from desk_cleaning.simulation import rotate_vector
from desk_cleaning.srv import SetSimGrasp, SetSimGraspResponse


class SimGraspManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.object_names = set(rospy.get_param('~objects', {}).keys())
        self.grasp_link = rospy.get_param('~grasp_link', 'robot::grasping_frame')
        self.fallback_link = rospy.get_param('~fallback_grasp_link', 'robot::link5')
        self.fallback_offset = rospy.get_param('~fallback_grasp_offset', [0.0, 0.0, 0.1126])
        self.max_distance = float(rospy.get_param('~max_attach_distance', 0.065))
        self.link_poses = {}
        self.model_poses = {}
        self.attached = None
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        rospy.Service('/desk_cleaning/sim_grasp', SetSimGrasp, self.handle_grasp)
        rate = max(5.0, float(rospy.get_param('~grasp_rate', 30.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.follow_object)
        rospy.loginfo('Gazebo logical grasp manager ready')

    def link_states_callback(self, msg):
        with self.lock:
            self.link_poses = dict(zip(msg.name, msg.pose))

    def model_states_callback(self, msg):
        with self.lock:
            self.model_poses = dict(zip(msg.name, msg.pose))

    def hold_pose(self):
        pose = self.link_poses.get(self.grasp_link)
        if pose is not None:
            return copy.deepcopy(pose)
        pose = self.link_poses.get(self.fallback_link)
        if pose is None:
            return None
        pose = copy.deepcopy(pose)
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        dx, dy, dz = rotate_vector(self.fallback_offset, quaternion)
        pose.position.x += dx
        pose.position.y += dy
        pose.position.z += dz
        return pose

    def handle_grasp(self, request):
        with self.lock:
            if not request.attach:
                released = self.attached
                self.attached = None
                return SetSimGraspResponse(True, 'released %s' % (released or 'nothing'))
            if request.object_id not in self.object_names:
                return SetSimGraspResponse(False, 'unknown simulated object: ' + request.object_id)
            object_pose = self.model_poses.get(request.object_id)
            hold_pose = self.hold_pose()
            if object_pose is None or hold_pose is None:
                return SetSimGraspResponse(False, 'Gazebo poses are not ready')
            distance = math.sqrt(
                (object_pose.position.x - hold_pose.position.x) ** 2
                + (object_pose.position.y - hold_pose.position.y) ** 2
                + (object_pose.position.z - hold_pose.position.z) ** 2
            )
            if distance > self.max_distance:
                return SetSimGraspResponse(
                    False, 'object is %.3fm from gripper (limit %.3fm)' % (distance, self.max_distance),
                )
            self.attached = request.object_id
            return SetSimGraspResponse(True, 'attached ' + request.object_id)

    def follow_object(self, _event):
        with self.lock:
            object_id = self.attached
            pose = self.hold_pose()
        if object_id is None or pose is None:
            return
        state = ModelState()
        state.model_name = object_id
        state.pose = pose
        state.reference_frame = 'world'
        try:
            response = self.set_model_state(state)
            if not response.success:
                rospy.logwarn_throttle(2.0, 'failed to move held object: %s', response.status_message)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, 'set_model_state failed: %s', exc)


if __name__ == '__main__':
    rospy.init_node('sim_grasp_manager')
    SimGraspManager()
    rospy.spin()
