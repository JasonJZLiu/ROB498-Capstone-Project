#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray
from std_srvs.srv import Empty, EmptyResponse

from configs import Configs
from rob498_drone.srv import WaypointEnqueueService
from waypoint_controller import WaypointController

from rob498_drone.srv import WorldToOdomViconService, WorldToOdomViconServiceRequest


class Task3:
    def __init__(self):
        rospy.wait_for_service("waypoint/enqueue")
        self.waypoint_enqueue_client = rospy.ServiceProxy("waypoint/enqueue", WaypointEnqueueService)

        rospy.wait_for_service("waypoint/takeoff")
        self.waypoint_takeoff_client = rospy.ServiceProxy("waypoint/takeoff", Empty)

        rospy.wait_for_service("waypoint/land")
        self.waypoint_land_client = rospy.ServiceProxy("waypoint/land", Empty)

        rospy.wait_for_service("waypoint/abort")
        self.waypoint_abort_client = rospy.ServiceProxy("waypoint/abort", Empty)

        name = 'rob498_drone_' + Configs.team_id
        srv_launch = rospy.Service(name + '/comm/launch', Empty, self._handle_launch_srv)
        srv_test = rospy.Service(name + '/comm/test', Empty, self._handle_test_srv)
        srv_land = rospy.Service(name + '/comm/land', Empty, self._handle_land_srv)
        srv_abort = rospy.Service(name + '/comm/abort', Empty, self._handle_abort_srv)

        # wait until we receive vicon data
        rospy.loginfo("Task3: Waiting for vicon data.")
        try:
            rospy.wait_for_message("vicon/ROB498_Drone/ROB498_Drone", TransformStamped)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")
        rospy.loginfo("Task3: Finished waiting for vicon data.")

        # wait until we receive the waypoints
        name = 'rob498_drone_' + Configs.team_id
        rospy.loginfo("Task3: Waiting for waypoints.")
        try:
            self.waypoints = rospy.wait_for_message(name + "/comm/waypoints", PoseArray)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for waypoints")
        rospy.loginfo("Task3: Finished waiting for waypoints.")

        
    def _handle_launch_srv(self, req):
        # calibrate world_to_odom using the given vicon pose
        rospy.wait_for_service("world_to_odom/vicon_calibrate")

        try:
            world_vicon = rospy.wait_for_message("vicon/ROB498_Drone/ROB498_Drone", TransformStamped)
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")

        try:
            self.srv_vicon_calibrate_client = rospy.ServiceProxy("world_to_odom/vicon_calibrate", WorldToOdomViconService)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        self.srv_vicon_calibrate_client(world_vicon)

        self.waypoint_takeoff_client()
        return EmptyResponse()


    def _handle_test_srv(self, req):
        waypoint_list = list()
        for waypoint_pose in self.waypoints.poses:
            waypoint_pose_stamped = PoseStamped()
            waypoint_pose_stamped.header.stamp = rospy.Time.now()
            waypoint_pose_stamped.header.frame_id = "vicon"
            waypoint_pose_stamped.pose = waypoint_pose
            waypoint_list.append(waypoint_pose_stamped)
        
        self.waypoint_enqueue_client(waypoint_list)

        return EmptyResponse()


    def _handle_land_srv(self, req):
        self.waypoint_land_client()
        return EmptyResponse()


    def _handle_abort_srv(self, req):
        self.waypoint_abort_client()
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node("task_3")
    task_3 = Task3()
    rospy.spin()





    
