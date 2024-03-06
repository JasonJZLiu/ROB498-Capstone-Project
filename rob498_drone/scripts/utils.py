#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion

import numpy as np
from typing import Union


def pose_to_transform_stamped(pose: Pose, frame_id="parent_frame", child_frame_id="child_frame"):
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = frame_id
    transform_stamped.child_frame_id = child_frame_id

    transform_stamped.transform.translation.x = pose.position.x
    transform_stamped.transform.translation.y = pose.position.y
    transform_stamped.transform.translation.z = pose.position.z

    transform_stamped.transform.rotation.x = pose.orientation.x
    transform_stamped.transform.rotation.y = pose.orientation.y
    transform_stamped.transform.rotation.z = pose.orientation.z
    transform_stamped.transform.rotation.w = pose.orientation.w

    return transform_stamped


# def transform_stamped_to_pose_stamped(tran: Pose, frame_id="parent_frame", child_frame_id="child_frame"):
#     transform_stamped = TransformStamped()
#     transform_stamped.header.stamp = rospy.Time.now()
#     transform_stamped.header.frame_id = frame_id
#     transform_stamped.child_frame_id = child_frame_id

#     transform_stamped.transform.translation.x = pose.position.x
#     transform_stamped.transform.translation.y = pose.position.y
#     transform_stamped.transform.translation.z = pose.position.z

#     transform_stamped.transform.rotation.x = pose.orientation.x
#     transform_stamped.transform.rotation.y = pose.orientation.y
#     transform_stamped.transform.rotation.z = pose.orientation.z
#     transform_stamped.transform.rotation.w = pose.orientation.w

#     return transform_stamped


def transform_stamped_to_odometry(transform_stamped: TransformStamped, frame_id, child_frame_id):
    odom = Odometry()
    odom.header = transform_stamped.header
    odom.header.stamp=rospy.Time.now()
    odom.header.frame_id = frame_id
    odom.child_frame_id = child_frame_id
    
    odom.pose.pose.position.x = transform_stamped.transform.translation.x
    odom.pose.pose.position.y = transform_stamped.transform.translation.y
    odom.pose.pose.position.z = transform_stamped.transform.translation.z
    odom.pose.pose.orientation.x = transform_stamped.transform.rotation.x
    odom.pose.pose.orientation.y = transform_stamped.transform.rotation.y
    odom.pose.pose.orientation.z = transform_stamped.transform.rotation.z
    odom.pose.pose.orientation.w = transform_stamped.transform.rotation.w
    
    odom.pose.covariance = [0] * 36
    
    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.y = 0
    odom.twist.twist.angular.z = 0
    
    odom.twist.covariance = [0] * 36
    
    return odom


def pose_to_drone_pose(pose: Pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    quat = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    euler = euler_from_quaternion(quat)
    yaw = euler[2]

    return np.array([x, y, z, yaw])


def pose_error(
        pose_1: Pose, 
        pose_2: Pose,
        full_pose: bool=True,
    ):
    if full_pose:
        p1 = np.array([pose_1.position.x, pose_1.position.y, pose_1.position.z])
        p2 = np.array([pose_2.position.x, pose_2.position.y, pose_2.position.z])
        pos_error = np.linalg.norm(p1 - p2)

        q1 = np.array([pose_1.orientation.x, pose_1.orientation.y, pose_1.orientation.z, pose_1.orientation.w])
        q2 = np.array([pose_2.orientation.x, pose_2.orientation.y, pose_2.orientation.z, pose_2.orientation.w])
        q1_conjugate = quaternion_inverse(q1)
        q_error = quaternion_multiply(q1_conjugate, q2)
        angle = 2*np.arctan2(np.linalg.norm(q_error[0:3]), q_error[3])
        rot_error = abs((angle + np.pi) % (2 * np.pi) - np.pi)
    else:
        pose_1 = pose_to_drone_pose(pose_1)
        pose_2 = pose_to_drone_pose(pose_2)

        pos_error = np.linalg.norm(pose_1[0:3] - pose_2[0:3])

        rot_error = pose_1[3] - pose_2[3]
        rot_error = np.arctan2(np.sin(rot_error), np.cos(rot_error))
        rot_error = np.abs(rot_error)

    return pos_error, rot_error




