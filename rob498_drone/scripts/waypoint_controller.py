#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from configs import Configs
from utils import pose_error
from rob498_drone.srv import WaypointEnqueueService, WaypointEnqueueServiceResponse


class WaypointController:
    has_taken_off = False

    def __init__(self):
        # subscribe to the current mavros state
        self.mavros_state = State()
        self.mavros_state_sub = rospy.Subscriber("mavros/state", State, callback=self._mavros_state_callback)

        # publisher for sending waypoints
        self.position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # subscriber for drone pose
        self.current_pose = PoseStamped()
        self.initial_pose = rospy.wait_for_message("mavros/local_position/pose", PoseStamped)
        self.land_height = self.initial_pose.pose.position.z
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self._pose_callback)

        self.setup_mavros_states()

        # a queue of PoseStamped waypoint
        self.waypoint_queue = list()
        self.current_waypoint = PoseStamped()
        self.setup_waypoint_srvs()

        return


    def setup_mavros_states(self):
        # client for checking and setting the arming state of the drone
        rospy.loginfo("Waiting for /mavros/cmd/arming")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.mavros_arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.loginfo("Finished waiting for /mavros/cmd/arming")

        # client for checking and setting the mode of the drone
        rospy.loginfo("Waiting for /mavros/set_mode")
        rospy.wait_for_service("/mavros/set_mode")
        self.mavros_set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        rospy.loginfo("Finished waiting for /mavros/set_mode")

        # setpoint publishing must be faster than 2Hz
        self.rate = rospy.Rate(60)

        # wait for flight controller connection
        rospy.loginfo("Connecting...")
        while(not rospy.is_shutdown() and not self.mavros_state.connected):
            self.rate.sleep()
        rospy.loginfo("Connected")
        return
    

    def _mavros_state_callback(self, state):
        self.mavros_state = state


    def _pose_callback(self, pose_stamped):
        self.current_pose = pose_stamped


    def setup_waypoint_srvs(self):
        # expose waypoint queue such that other nodes can use it 
        rospy.Service("waypoint/enqueue", WaypointEnqueueService, self._handle_waypoint_enqueue_srv)
        rospy.Service("waypoint/clear", Empty, self._handle_waypoint_clear_srv)

        # set up takeoff, land, and abort commands
        rospy.Service("waypoint/takeoff", Empty, self._handle_takeoff_srv)
        rospy.Service("waypoint/land", Empty, self._handle_land_srv)
        rospy.Service("waypoint/abort", Empty, self._handle_abort_srv)


    def _handle_waypoint_enqueue_srv(self, req):
        self.waypoint_queue += req.poses
        return WaypointEnqueueServiceResponse(success=True, message="Waypoint Enqueue Processed successfully")


    def _handle_waypoint_clear_srv(self, req):
        self.waypoint_queue = list()
        return EmptyResponse()


    def _handle_takeoff_srv(self, req):
        take_off_pose = PoseStamped()
        take_off_pose.pose.position.x = 0
        take_off_pose.pose.position.y = 0
        take_off_pose.pose.position.z = 1.5 - 0.13
        take_off_pose.pose.orientation.x = 0
        take_off_pose.pose.orientation.y = 0
        take_off_pose.pose.orientation.z = 0
        take_off_pose.pose.orientation.w = 1.0

        # must start streaming waypoints before entering OFFBOARD mode
        for i in range(100):
            if(rospy.is_shutdown()):
                break
            self.position_pub.publish(take_off_pose)
            self.rate.sleep()
    
        # set drone mode to OFFBOARD
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = "OFFBOARD"
        if self.mavros_set_mode_client.call(offb_set_mode).mode_sent == True:
            rospy.loginfo("OFFBOARD enabled.")
        else:
            raise rospy.ROSException("Failed to set the drone mode to OFFBOARD.")
        
        # arm the drone
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if self.mavros_arming_client.call(arm_cmd).success == True:
            rospy.loginfo("Drone armed.")
        else:
            raise rospy.ROSException("Failed to arm drone.")

        rospy.loginfo("Launching drone.")
        self.current_waypoint = take_off_pose
        self.has_taken_off = True

        return EmptyResponse()
    

    def _handle_land_srv(self, req):
        land_pose = PoseStamped()
        land_pose.pose.position.x = self.current_pose.pose.position.x
        land_pose.pose.position.y = self.current_pose.pose.position.y
        land_pose.pose.position.z = self.land_height
        land_pose.pose.orientation.x = 0
        land_pose.pose.orientation.y = 0
        land_pose.pose.orientation.z = 0
        land_pose.pose.orientation.w = 1.0

        if len(self.waypoint_queue) == 0:
            rospy.loginfo("Landing drone.")
            self.current_waypoint = land_pose
            while(not rospy.is_shutdown() or not self.mavros_state.armed):
                if self.current_pose.pose.position.z < self.land_height + 0.1:
                    break
                self.position_pub.publish(self.current_waypoint)
                self.rate.sleep()
                
            self.has_taken_off = False
            rospy.loginfo("Drone landed successfully.")            
        else:
            rospy.logwarn("Waypoint queue is not empty, cannot land!")

        return EmptyResponse()


    def _handle_abort_srv(self, req):
        # self.has_taken_off == False
        # set drone mode to STABILIZED
        set_mode = SetModeRequest()
        set_mode.custom_mode = "STABILIZED"
        if self.mavros_set_mode_client.call(set_mode).mode_sent == True:
            rospy.loginfo("STABILIZED enabled.")
        else:
            # raise rospy.ROSException("Failed to set the drone mode to STABILIZED.")
            rospy.logwarn("Failed to set the drone mode to STABILIZED.")

    

    def run(self):
        while(not rospy.is_shutdown()):
            if self.has_taken_off == False:
                self.rate.sleep()
                continue
            
            pos_error, rot_error = pose_error(
                self.current_pose.pose, 
                self.current_waypoint.pose, 
                full_pose=True,
            )
            
            if pos_error < Configs.waypoint_pos_tol and rot_error < Configs.waypoint_rot_tol:
                rospy.loginfo("Current waypoint reached.") 
                if len(self.waypoint_queue) > 0:
                    self.current_waypoint = self.waypoint_queue.pop(0)
                    rospy.loginfo("Setting the next waypoint.")
 
            self.position_pub.publish(self.current_waypoint)
            self.rate.sleep()
            


if __name__ == "__main__":
    rospy.init_node('waypoint_controller')
    waypoint_controller = WaypointController()
    waypoint_controller.run()
