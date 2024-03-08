#!/usr/bin/env python3

import rospy
import tf2_ros
import copy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from utils import transform_stamped_to_odometry, transform_stamped_to_matrix, odom_to_matrix, matrix_to_pose


class RST265Bridge:
    def __init__(self):
        # wait until we receive vicon data
        # try:
        #     rospy.wait_for_message("vicon/ROB498_Drone/ROB498_Drone", TransformStamped)
        # except rospy.ROSException as e:
        #     rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # odom = rospy.wait_for_message("/camera/odom/sample_throttled", Odometry)
        # odom_time = 

        # try:
        #     self.baselink_T_camera = self.tf_buffer.lookup_transform(
        #         "base_link",
        #         "camera_pose_frame",
        #         rospy.Time.now(),
        #     )
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Cannot retrieve static transform between base_link and camera_pose_frame.")

        self.baselink_T_camera = TransformStamped()
        self.baselink_T_camera.header.stamp = rospy.Time.now()
        self.baselink_T_camera.header.frame_id = "base_link"
        self.baselink_T_camera.child_frame_id = "camera_pose_frame"

        # self.baselink_T_camera.transform.translation.x = 0.077
        # self.baselink_T_camera.transform.translation.y = -0.085
        # self.baselink_T_camera.transform.translation.z = -0.085

        self.baselink_T_camera.transform.translation.x = 0.077
        self.baselink_T_camera.transform.translation.y = -0.085
        self.baselink_T_camera.transform.translation.z = -0.085
        self.baselink_T_camera.transform.rotation.x = 0
        self.baselink_T_camera.transform.rotation.y = 0.707
        self.baselink_T_camera.transform.rotation.z = 0
        self.baselink_T_camera.transform.rotation.w = 0.707

        baselink_T_camera_matrix = transform_stamped_to_matrix(self.baselink_T_camera)
        self.camera_T_baselink = np.linalg.inv(baselink_T_camera_matrix)

        
        self.rs_t265_sub = rospy.Subscriber(
            "/camera/odom/sample_throttled", Odometry, callback=self._rs_t265_sub_callback
        )
        self.mavros_odometry_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)


    def _rs_t265_sub_callback(self, odom):
        # odom: camera_odom_frame -> camera_pose_frame
        # baselink_T_camera_pose_frame
        # want: camera_odom_Frame -> base_link

        odom_T_camera = odom_to_matrix(odom)
        odom_T_baselink = odom_T_camera @ self.camera_T_baselink

        odom_baselink = Odometry()
        odom_baselink.header.stamp = rospy.Time.now()
        odom_baselink.header.frame_id = "odom"
        odom_baselink.child_frame_id = "base_link"

        odom_baselink.pose.pose = matrix_to_pose(odom_T_baselink)

        self.mavros_odometry_pub.publish(odom_baselink)




if __name__ == '__main__':
    rospy.init_node('rs_t265_bridge')
    vicon_sim = RST265Bridge()
    rospy.spin()
