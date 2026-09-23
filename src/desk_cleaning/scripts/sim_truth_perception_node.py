#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool

from desk_cleaning.msg import DetectedObject, DetectedObjectArray


class SimTruthPerception:
    def __init__(self):
        self.objects = rospy.get_param('~objects', {})
        self.destination_regions = rospy.get_param('~destination_regions', [])
        self.objects_pub = rospy.Publisher('~objects_out', DetectedObjectArray, queue_size=1)
        self.ready_pub = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        rospy.loginfo('Gazebo ground-truth perception waiting for model states')

    def is_collected(self, pose):
        for region in self.destination_regions:
            distance = math.hypot(pose.position.x - float(region['x']), pose.position.y - float(region['y']))
            if distance <= float(region['radius']):
                return True
        return False

    def model_states_callback(self, msg):
        poses = dict(zip(msg.name, msg.pose))
        output = DetectedObjectArray()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = 'base_link'
        for model_name, config in self.objects.items():
            pose = poses.get(model_name)
            if pose is None or self.is_collected(pose):
                continue
            item = DetectedObject()
            item.id = model_name
            item.class_name = config['class_name']
            item.display_name = config.get('display_name', config['class_name'])
            item.confidence = 1.0
            item.position = pose.position
            item.grasp_yaw = float(config.get('grasp_yaw', 0.0))
            size = config.get('size', [0.03, 0.03, 0.02])
            item.size_x, item.size_y, item.size_z = (float(value) for value in size)
            item.valid_depth = True
            output.objects.append(item)
        self.objects_pub.publish(output)
        self.ready_pub.publish(Bool(data=True))


if __name__ == '__main__':
    rospy.init_node('sim_truth_perception')
    SimTruthPerception()
    rospy.spin()
