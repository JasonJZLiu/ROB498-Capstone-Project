#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from configs import Configs
from rob498_drone.srv import WaypointEnqueueService, WaypointEnqueueServiceResponse


class WaypointController:
    def __init__(self):
        # subscribe to the current mavros state
        self.mavros_state = State()
        self.mavros_state_sub = rospy.Subscriber("mavros/state", State, callback=self.mavros_state_callback)

        # publisher for sending waypoints
        self.position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # subscriber for drone pose
        self.current_pose = PoseStamped()
        self.initial_pose = rospy.wait_for_message("mavros/local_position/pose", PoseStamped, timeout=30)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self.pose_callback)

        self._setup_mavros_states()

        # a queue of PoseStamped waypoint
        self.waypoint_queue = list()
        self._setup_waypoint_srvs()

        return


    def _setup_mavros_states(self):
        # client for checking and setting the arming state of the drone
        rospy.wait_for_service("/mavros/cmd/arming")
        self.mavros_arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        # client for checking and setting the mode of the drone
        rospy.wait_for_service("/mavros/set_mode")
        self.mavros_set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # setpoint publishing must be faster than 2Hz
        self.rate = rospy.Rate(20)

        # wait for flight controller connection
        while(not rospy.is_shutdown() and not self.mavros_state.connected):
            self.rate.sleep()

        # must start streaming waypoints before entering offboard mode
        for i in range(100):
            if(rospy.is_shutdown()):
                break
            self.position_pub.publish(self.initial_pose)
            self.rate.sleep()
    
        # set drone mode to offboard
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = "OFFBOARD"
        if self.mavros_set_mode_client.call(offb_set_mode).mode_sent == True:
            rospy.loginfo("OFFBOARD enabled")
        else:
            raise rospy.ROSException("Failed to set the drone mode to OFFBOARD.")
        
        # arm the drone automatically if configured
        if Configs.automatic_arming:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            if self.mavros_arming_client.call(arm_cmd).success == True:
                rospy.loginfo("Drone armed")
            else:
                raise rospy.ROSException("Failed to arm drone automatically.")

        return
    

    def mavros_state_callback(self, state):
        self.mavros_state = state


    def pose_callback(self, pose_stamped):
        self.current_pose = pose_stamped




    def _setup_waypoint_srvs(self):
        s = rospy.Service('waypoint/enqueue', WaypointEnqueueService, self.handle_waypoint_enqueue_srv)
    

    def handle_waypoint_enqueue_srv(self, req):
        print(req.poses)
        # self.waypoint_queue += req.poses
        return WaypointEnqueueServiceResponse(success=True, message="Waypoint Enqueue Processed successfully")



    


    def run(self):
        last_status_check_time = rospy.Time.now()
        while(not rospy.is_shutdown()):
            if (rospy.Time.now() - last_status_check_time) > rospy.Duration(5.0):
                if self.mavros_state.mode != "OFFBOARD":
                    rospy.logwarn("Drone is not in OFFBOARD mode!")
                elif self.mavros_state.armed == False:
                    rospy.logwarn("Drone is not armed!")
                last_status_check_time = rospy.Time.now()

            
            pose = PoseStamped()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2

            self.position_pub.publish(pose)
            self.rate.sleep()
            





if __name__ == "__main__":
    rospy.init_node('waypoint_controller')
    waypoint_controller = WaypointController()
    waypoint_controller.run()
