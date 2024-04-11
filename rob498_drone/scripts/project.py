#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Point
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

from configs import Configs
from rob498_drone.srv import AStarService
from waypoint_controller import WaypointController

from rob498_drone.srv import ProjectTeleopService, ProjectTeleopServiceResponse
from rob498_drone.srv import WaypointEnqueueService

import numpy as np
from tf.transformations import euler_matrix

from utils import *



class Project:
    def __init__(self):
        self.enable_path_planner = False

        # 1 hz
        self.rate = rospy.Rate(1)

        rospy.wait_for_service("waypoint/enqueue")
        self.waypoint_enqueue_client = rospy.ServiceProxy("waypoint/enqueue", WaypointEnqueueService)

        rospy.wait_for_service("waypoint/clear")
        self.waypoint_clear_client = rospy.ServiceProxy("waypoint/clear", Empty)

        rospy.wait_for_service("path_planner/run_astar")
        self.path_planner_run_astar_client = rospy.ServiceProxy("path_planner/run_astar", AStarService)

        rospy.wait_for_message("mavros/local_position/pose", PoseStamped)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self._pose_callback)

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
        srv_teleop = rospy.Service(name + '/comm/teleop', ProjectTeleopService, self._handle_teleop_srv)


        self.target_point_marker_pub = rospy.Publisher('/project_target_point_marker', Marker, queue_size=10)
        self.safe_target_point_marker_pub = rospy.Publisher('/project_safe_target_point_marker', Marker, queue_size=10)
        self.waypoint_path_publisher = rospy.Publisher('/project_waypoint_path', Path, queue_size=10)

     
    def _pose_callback(self, pose_stamped):
        # in frame: map
        self.drone_pose = pose_stamped


    def _handle_launch_srv(self, req):
        self.waypoint_takeoff_client()
        return EmptyResponse()
        

    def _handle_land_srv(self, req):
        self.waypoint_land_client()
        return EmptyResponse()


    def _handle_abort_srv(self, req):
        self.waypoint_abort_client()
        return EmptyResponse()


    def _handle_teleop_srv(self, req):
        large_trans_delta_x = 2.0
        trans_delta_x = 1.0
        trans_delta_y = 1.0
        trans_delta_z = 0.3
        yaw_delta = 0.3926991

        teleop_command = req.teleop_command

        odom_T_base = pose_stamped_to_matrix(self.drone_pose)
        base_T_target = np.eye(4)

        if teleop_command == "w":
            base_T_target[0, 3] = trans_delta_x
        elif teleop_command == "s":
            base_T_target[0, 3] = -trans_delta_x
        elif teleop_command == "a":
            base_T_target[1, 3] = trans_delta_y
        elif teleop_command == "d":
            base_T_target[1, 3] = -trans_delta_y
        elif teleop_command == "q":
            base_T_target[2, 3] = trans_delta_z
        elif teleop_command == "e":
            base_T_target[2, 3] = -trans_delta_z
        elif teleop_command == "j":
            base_T_target = euler_matrix(0, 0, yaw_delta)
        elif teleop_command == "l":
            base_T_target = euler_matrix(0, 0, -yaw_delta)
        elif teleop_command == "g":
            base_T_target[0, 3] = large_trans_delta_x

        odom_T_target = odom_T_base @ base_T_target
        target_pose = matrix_to_pose(odom_T_target)
        self.target_point = target_pose.position
        self.visualize_target_point(self.target_point)

        if teleop_command in ["j", "l", "q", "e"]:
            waypoint_list = list()
            waypoint_pose_stamped = PoseStamped()
            waypoint_pose_stamped.header.stamp = rospy.Time.now()
            waypoint_pose_stamped.header.frame_id = "vicon"
            waypoint_pose_stamped.pose = target_pose
            waypoint_list = [waypoint_pose_stamped]
            
            self.enable_path_planner = False
            self.waypoint_clear_client()
            self.waypoint_enqueue_client(waypoint_list)
            self.clear_visualization()
            


        elif teleop_command == "x":
            self.enable_path_planner = False
            self.waypoint_clear_client()
            self.clear_visualization()
        else:
            self.enable_path_planner = True


        return ProjectTeleopServiceResponse(success=True)


    def clear_visualization(self):
        marker = Marker()
        marker.header.frame_id = "world"  # Same frame_id as the original marker
        marker.ns = "spheres"  # Same namespace as the original marker
        marker.id = 0  # Same ID as the original marker
        marker.action = Marker.DELETE  # Action to delete the marker
        self.safe_target_point_marker_pub.publish(marker)

        empty_path = Path()
        empty_path.header.frame_id = "world"
        empty_path.header.stamp = rospy.Time.now()
        self.waypoint_path_publisher.publish(empty_path)



    def visualize_target_point(self, target_point):
        # visualization
        marker = Marker()
        marker.header.frame_id = "world"  # Adjust based on your TF frames
        marker.header.stamp = rospy.Time.now()
        marker.ns = "spheres"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target_point.x
        marker.pose.position.y = target_point.y
        marker.pose.position.z = target_point.z
        marker.pose.orientation.w = 1.0
        
        # Size of the sphere
        sphere_diameter = 0.2  # Adjust the size of the sphere here
        marker.scale.x = sphere_diameter  # Diameter of the sphere in X direction
        marker.scale.y = sphere_diameter  # Diameter of the sphere in Y direction
        marker.scale.z = sphere_diameter  # Diameter of the sphere in Z direction
        
        # Color and transparency
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
                
        self.target_point_marker_pub.publish(marker)
    

    def run(self):
        while(not rospy.is_shutdown()):
            if self.enable_path_planner == False:
                self.rate.sleep()
                continue

            self.path_planner_run_astar_client(self.target_point)
            self.rate.sleep()
    


if __name__ == '__main__':
    rospy.init_node("Project")
    project = Project()
    project.run()
    rospy.spin()





    
