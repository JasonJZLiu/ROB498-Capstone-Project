#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from utils import transform_stamped_to_odometry


class ViconBridge:
    def __init__(self):
        # wait until we receive vicon data
        try:
            rospy.wait_for_message("vicon/ROB498_Drone/ROB498_Drone", TransformStamped)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")

        self.vicon_sub = rospy.Subscriber(
            "vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback=self._vicon_sub_callback
        )
        # self.mavros_odometry_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)
        self.mavros_odometry_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)


    def _vicon_sub_callback(self, transform_stamped):
        # odom = transform_stamped_to_odometry(
        #     transform_stamped=transform_stamped,
        #     frame_id="odom",
        #     child_frame_id="base_link",
        # )
        # self.mavros_odometry_pub.publish(odom)

        pose = PoseStamped()
        pose.pose.position.x = transform_stamped.transform.translation.x 
        pose.pose.position.y = transform_stamped.transform.translation.y
        pose.pose.position.z = transform_stamped.transform.translation.z
        pose.pose.orientation.x = transform_stamped.transform.rotation.x 
        pose.pose.orientation.y = transform_stamped.transform.rotation.y
        pose.pose.orientation.z = transform_stamped.transform.rotation.z 
        pose.pose.orientation.w = transform_stamped.transform.rotation.w 
        self.mavros_odometry_pub.publish(pose)



     

if __name__ == '__main__':
    rospy.init_node('vicon_bridge')
    vicon_sim = ViconBridge()
    rospy.spin()
