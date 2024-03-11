#!/usr/bin/env python3

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from utils import *


class ViconBridge:
    def __init__(self):

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(1)

        # wait until we receive vicon data
        try:
            rospy.wait_for_message("vicon/ROB498_Drone/ROB498_Drone", TransformStamped)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")

        while not self.tf_buffer.can_transform('vicon', 'base_link', rospy.Time(0)):
            rospy.loginfo("ViconBridge: Waiting for base_link to vicon transform!")
            self.rate.sleep()
        self.vicon_T_base = transform_stamped_to_matrix(self.tf_buffer.lookup_transform('vicon', 'base_link', rospy.Time(0)))


        # TODO: wait for world to odom
        
        
        rospy.loginfo("ViconBridge: Finished waiting! Starting ViconBridge.")

        self.vicon_sub = rospy.Subscriber(
            "vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback=self._vicon_sub_callback
        )
        self.mavros_odometry_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)


    def _vicon_sub_callback(self, transform_stamped):
        # transform_stamped: world -> vicon
        # transform to odom -> vicon
        # then transform to odom-> baseLink


        # transform_stamped.header.frame_id = "world"
        # transform_stamped.child_frame_id = "vicon"
        # self.tf_pub.sendTransform(transform_stamped)
        # odom = transform_stamped_to_odometry(
        #     transform_stamped=self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0)),
        #     frame_id="odom",
        #     child_frame_id="base_link",
        # )

        world_T_vicon = transform_stamped_to_matrix(transform_stamped)
        odom_T_world = transform_stamped_to_matrix(self.tf_buffer.lookup_transform('odom', 'world', rospy.Time(0)))
        odom_T_base_link = odom_T_world @ world_T_vicon @ self.vicon_T_base

        odom = Odometry()
        odom.header = transform_stamped.header
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = matrix_to_pose(odom_T_base_link)


        # tf_odom_base_link = TransformStamped()
        # tf_odom_base_link.header = transform_stamped.header
        # tf_odom_base_link.header.frame_id = "odom"
        # tf_odom_base_link.child_frame_id = "base_link"
        # tf_odom_base_link.transform = matrix_to_transform(odom_T_base_link)
        # self.tf_pub.sendTransform(tf_odom_base_link)

        # odom = transform_stamped_to_odometry(
        #     transform_stamped=tf_odom_base_link,
        #     frame_id="odom",
        #     child_frame_id="base_link",
        # )

        self.mavros_odometry_pub.publish(odom)
    

if __name__ == '__main__':
    rospy.init_node('vicon_bridge')
    vicon_sim = ViconBridge()
    rospy.spin()
