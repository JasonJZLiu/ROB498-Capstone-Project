#!/usr/bin/env python3

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from utils import *
from rob498_drone.srv import WorldToOdomViconService, WorldToOdomViconServiceResponse


class WorldToOdom:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        # initially assume world frame aligns with odom frame
        self.world_odom = TransformStamped()
        self.world_odom.header.stamp = rospy.Time.now()
        self.world_odom.header.frame_id = "world"
        self.world_odom.child_frame_id = "odom"
        self.world_odom.transform.translation.x = 0
        self.world_odom.transform.translation.y = 0
        self.world_odom.transform.translation.z = 0
        self.world_odom.transform.rotation.x = 0
        self.world_odom.transform.rotation.y = 0
        self.world_odom.transform.rotation.z = 0
        self.world_odom.transform.rotation.w = 1
        self.tf_pub.sendTransform(self.world_odom)

        while not self.tf_buffer.can_transform('vicon', 'base_link', rospy.Time(0)):
            rospy.loginfo("WorldToOdom: Waiting for base_link to vicon transform!")
            rospy.sleep(0.1)
        self.vicon_T_base = transform_stamped_to_matrix(self.tf_buffer.lookup_transform('vicon', 'base_link', rospy.Time(0)))

        try:
            rospy.loginfo("WorldToOdom: Waiting for mavros/local_position/pose")
            rospy.wait_for_message("mavros/local_position/pose", PoseStamped)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")
        rospy.loginfo("WorldToOdom: Finished waiting!")

        rospy.Service("world_to_odom/vicon_calibrate", WorldToOdomViconService, self._handle_vicon_calibrate_srv)

        self.rate = rospy.Rate(100)
    

    def _handle_vicon_calibrate_srv(self, req):
        self.world_T_vicon = transform_stamped_to_matrix(req.world_vicon)

        odom_T_base = pose_stamped_to_matrix(rospy.wait_for_message("mavros/local_position/pose", PoseStamped))
        base_T_odom = np.linalg.inv(odom_T_base)

        self.world_T_odom = self.world_T_vicon @ self.vicon_T_base @ base_T_odom

        self.world_odom.header.stamp = rospy.Time.now()
        self.world_odom.transform = matrix_to_transform(self.world_T_odom)
        self.tf_pub.sendTransform(self.world_odom)

        return WorldToOdomViconServiceResponse(success=True, message="WorldToOdom Vicon Calibrate Service Processed successfully")


    def run(self):
        while (not rospy.is_shutdown()):
            self.world_odom.header.stamp = rospy.Time.now()
            self.tf_pub.sendTransform(self.world_odom)
            self.rate.sleep()



if __name__ == '__main__':
    rospy.init_node('world_to_odom')
    vicon_sim = WorldToOdom()
    vicon_sim.run()
