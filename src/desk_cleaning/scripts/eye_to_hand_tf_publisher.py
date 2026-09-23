#!/usr/bin/env python3
"""Publish a calibrated fixed-base RGB-D camera transform from JSON."""

import json

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def matrix_from_json(value):
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('base_to_camera must be a 4x4 matrix')
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-8):
        raise ValueError('base_to_camera has an invalid homogeneous row')
    return matrix


def transform_to_matrix(transform):
    quaternion = transform.rotation
    matrix = quaternion_matrix((
        quaternion.x, quaternion.y, quaternion.z, quaternion.w,
    ))
    translation = transform.translation
    matrix[:3, 3] = (translation.x, translation.y, translation.z)
    return matrix


def main():
    rospy.init_node('eye_to_hand_tf_publisher')
    result_path = rospy.get_param(
        '~result',
        '/home/ubuntu/share/tmp/ArmPi-FPV/output/eye_to_hand_result.json',
    )
    base_frame = rospy.get_param('~base_frame', 'base_link')
    camera_root_frame = rospy.get_param('~camera_root_frame', 'astra_link')
    camera_optical_frame = rospy.get_param(
        '~camera_optical_frame', 'astra_color_optical_frame',
    )

    with open(result_path, 'r', encoding='utf-8') as stream:
        result = json.load(stream)
    result_base_frame = result.get('base_frame')
    if result_base_frame and result_base_frame != base_frame:
        raise ValueError(
            'result base frame {} does not match {}'.format(
                result_base_frame, base_frame,
            )
        )
    base_to_optical = matrix_from_json(result['base_to_camera'])

    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    listener = tf2_ros.TransformListener(buffer)
    rospy.loginfo(
        'waiting for camera internal TF %s -> %s',
        camera_root_frame, camera_optical_frame,
    )
    internal = None
    while not rospy.is_shutdown() and internal is None:
        try:
            internal = buffer.lookup_transform(
                camera_root_frame,
                camera_optical_frame,
                rospy.Time(0),
                rospy.Duration(5.0),
            )
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(
                10.0, 'still waiting for camera internal TF: %s', exc,
            )
    if internal is None:
        return
    root_to_optical = transform_to_matrix(internal.transform)
    base_to_root = base_to_optical.dot(np.linalg.inv(root_to_optical))

    message = TransformStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = base_frame
    message.child_frame_id = camera_root_frame
    message.transform.translation.x = float(base_to_root[0, 3])
    message.transform.translation.y = float(base_to_root[1, 3])
    message.transform.translation.z = float(base_to_root[2, 3])
    quaternion = quaternion_from_matrix(base_to_root)
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(message)
    rospy.loginfo(
        'published calibrated TF %s -> %s from %s',
        base_frame, camera_root_frame, result_path,
    )
    rospy.spin()


if __name__ == '__main__':
    main()
