#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

from configs import Configs
from rob498_drone.srv import WaypointEnqueueService
from waypoint_controller import WaypointController


class Task2:
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

        
    def _handle_launch_srv(self, req):
        self.waypoint_takeoff_client()
        return EmptyResponse()


    def _handle_test_srv(self, req):
        return EmptyResponse()


    def _handle_land_srv(self, req):
        self.waypoint_land_client()
        return EmptyResponse()


    def _handle_abort_srv(self, req):
        self.waypoint_abort_client()
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node("task_2")
    task_2 = Task2()
    rospy.spin()





    
