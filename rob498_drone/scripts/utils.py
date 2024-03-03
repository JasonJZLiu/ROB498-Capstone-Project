#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TransformStamped


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