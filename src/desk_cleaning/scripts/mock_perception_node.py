#!/usr/bin/env python3
"""Publish deterministic objects for a no-camera, no-arm integration demo."""
import rospy
from std_msgs.msg import Bool

from desk_cleaning.msg import DetectedObject, DetectedObjectArray


class MockPerception:
    def __init__(self):
        self.frame = rospy.get_param('~frame_id', 'base_link')
        self.objects = rospy.get_param('~objects', [
            {'id': 'mock-pen', 'class_name': 'pen', 'display_name': '签字笔',
             'x': 0.16, 'y': 0.04, 'z': 0.02, 'size_x': 0.14, 'size_y': 0.015},
            {'id': 'mock-eraser', 'class_name': 'eraser', 'display_name': '橡皮',
             'x': 0.18, 'y': -0.05, 'z': 0.02, 'size_x': 0.04, 'size_y': 0.025},
        ])
        self.publisher = rospy.Publisher('~objects_out', DetectedObjectArray, queue_size=1)
        self.ready = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)
        self.ready.publish(Bool(data=True))

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = DetectedObjectArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame
            for config in self.objects:
                item = DetectedObject()
                item.id = config['id']
                item.class_name = config['class_name']
                item.display_name = config.get('display_name', item.class_name)
                item.confidence = float(config.get('confidence', 0.95))
                item.position.x = float(config['x'])
                item.position.y = float(config['y'])
                item.position.z = float(config['z'])
                item.size_x = float(config.get('size_x', 0.03))
                item.size_y = float(config.get('size_y', 0.03))
                item.size_z = float(config.get('size_z', 0.02))
                item.valid_depth = True
                msg.objects.append(item)
            self.publisher.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mock_desk_perception')
    MockPerception().spin()
