#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from configs import Configs
from utils import pose_to_transform_stamped


class ViconFake:
    def __init__(self):

        self.transform_stamped = TransformStamped()

        try:
            rospy.wait_for_message("mavros/odometry/in", Odometry)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for mavros/odometry/in.")

        self.mavros_odometry_sub = rospy.Subscriber("mavros/odometry/in", Odometry, self._mavros_odom_callback)
        self.vicon_sim_pub = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size=10)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(10)


    def _mavros_odom_callback(self, odom):
        try:
            self.transform_stamped = TransformStamped()
            self.transform_stamped.header.stamp = rospy.Time.now()
            self.transform_stamped.header.frame_id = odom.header.frame_id
            self.transform_stamped.child_frame_id = odom.child_frame_id

            self.transform_stamped.transform.translation.x = odom.pose.pose.position.x
            self.transform_stamped.transform.translation.y = odom.pose.pose.position.y
            self.transform_stamped.transform.translation.z = odom.pose.pose.position.z

            self.transform_stamped.transform.rotation.x = odom.pose.pose.orientation.x
            self.transform_stamped.transform.rotation.y = odom.pose.pose.orientation.y
            self.transform_stamped.transform.rotation.z = odom.pose.pose.orientation.z
            self.transform_stamped.transform.rotation.w = odom.pose.pose.orientation.w

        except ValueError:
            print(f"Vicon Fake Failed")


    def run(self):
        while not rospy.is_shutdown():
            self.vicon_sim_pub.publish(self.transform_stamped)
            self.tf_pub.sendTransform(self.transform_stamped)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('vicon_fake')
    vicon_sim = ViconFake()
    vicon_sim.run()
