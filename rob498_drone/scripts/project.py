#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Point
from std_srvs.srv import Empty, EmptyResponse

from configs import Configs
from rob498_drone.srv import AStarService
from waypoint_controller import WaypointController

from rob498_drone.srv import WorldToOdomViconService, WorldToOdomViconServiceRequest


class ProjectClass:
    def __init__(self):
        print("WAITING FOR path_planner/run_astar")
        rospy.wait_for_service("path_planner/run_astar")
        self.path_planner_run_astar_client = rospy.ServiceProxy("path_planner/run_astar", AStarService)


        rospy.wait_for_service("waypoint/takeoff")
        self.waypoint_takeoff_client = rospy.ServiceProxy("waypoint/takeoff", Empty)

        rospy.wait_for_service("waypoint/land")
        self.waypoint_land_client = rospy.ServiceProxy("waypoint/land", Empty)

        rospy.wait_for_service("waypoint/abort")
        self.waypoint_abort_client = rospy.ServiceProxy("waypoint/abort", Empty)


        name = 'rob498_drone_' + Configs.team_id
        srv_launch = rospy.Service(name + '/comm/launch', Empty, self._handle_launch_srv)
        srv_land = rospy.Service(name + '/comm/land', Empty, self._handle_land_srv)
        srv_abort = rospy.Service(name + '/comm/abort', Empty, self._handle_abort_srv)

     


    def _handle_launch_srv(self, req):
        self.waypoint_takeoff_client()
        return EmptyResponse()
        

    def _handle_land_srv(self, req):
        self.waypoint_land_client()
        return EmptyResponse()


    def _handle_abort_srv(self, req):
        self.waypoint_abort_client()
        return EmptyResponse()
    


    def run(self):
        while (not rospy.is_shutdown()):
            print("1: run a star")

            key = input()
            print(key)
            if key == '1':
                target_point = Point()
                target_point.x = 10 #0.3
                target_point.y = 0
                target_point.z =0

                self.path_planner_run_astar_client(target_point)
                print("A Star request sent successfully.")
            # elif key == '2':
            #     self.srv_test_client()
            #     print("Test request sent successfully.")
            # elif key == '3':
            #     self.srv_land_client()
            #     print("Land request sent successfully.")
            # elif key == '4':
            #     self.srv_abort_client()
            #     print("Abort request sent successfully.")
            else:
                print("Invalid input")




    


if __name__ == '__main__':
    rospy.init_node("project")
    project_asd = ProjectClass()
    project_asd.run()
    rospy.spin()





    
