#! /usr/bin/env python3

import rospy
import tf2_ros
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from configs import Configs
from utils import pose_error
from rob498_drone.srv import WaypointEnqueueService, WaypointEnqueueServiceResponse
import copy

from utils import *
import threading


class WaypointController:
    has_taken_off = False

    def __init__(self):
        self.waypoint_queue_mutex = threading.Lock()
        self.tf_pub = tf2_ros.TransformBroadcaster()

        # subscribe to the current mavros state
        self.mavros_state = State()
        self.mavros_state_sub = rospy.Subscriber("mavros/state", State, callback=self._mavros_state_callback)

        # publisher for sending waypoints
        self.position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # subscriber for drone pose
        self.current_pose = PoseStamped()
        self.initial_pose = rospy.wait_for_message("mavros/local_position/pose", PoseStamped)
        # in odom frame
        self.land_height = self.initial_pose.pose.position.z
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self._pose_callback)

        self.setup_mavros_states()

        self.setup_transforms()        

        # a queue of PoseStamped waypoint
        self.waypoint_queue = list()
        self.current_waypoint = PoseStamped()
        self.setup_waypoint_srvs()

        return


    def setup_transforms(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_pub = tf2_ros.TransformBroadcaster()

        while not self.tf_buffer.can_transform('vicon', 'base_link', rospy.Time(0)):
            rospy.loginfo("ViconBridge: Waiting for base_link to vicon transform!")
            self.rate.sleep()
        self.vicon_T_base = transform_stamped_to_matrix(self.tf_buffer.lookup_transform('vicon', 'base_link', rospy.Time(0)))

        while not self.tf_buffer.can_transform('odom', 'world', rospy.Time(0)):
            rospy.loginfo("ViconBridge: Waiting for odom to world transform!")
            self.rate.sleep()
        rospy.loginfo("WaypointController: Finished waiting for transforms!")



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
        # in frame: map
        self.current_pose = pose_stamped
        
        # optional for connecting odom to base_link in the tf_tree
        tf_odom_base_link = pose_to_transform_stamped(
            pose=pose_stamped.pose, frame_id="odom", child_frame_id="base_link"
        )
        self.tf_pub.sendTransform(tf_odom_base_link)




    def setup_waypoint_srvs(self):
        # expose waypoint queue such that other nodes can use it 
        rospy.Service("waypoint/enqueue", WaypointEnqueueService, self._handle_waypoint_enqueue_srv)
        rospy.Service("waypoint/clear", Empty, self._handle_waypoint_clear_srv)

        # set up takeoff, land, and abort commands
        rospy.Service("waypoint/takeoff", Empty, self._handle_takeoff_srv)
        rospy.Service("waypoint/land", Empty, self._handle_land_srv)
        rospy.Service("waypoint/abort", Empty, self._handle_abort_srv)


    def transform_waypoint_target(self, world_vicon: PoseStamped):
        # converts a waypoint target representing: world_T_vicon to
        # a PoseStamped representing: odom_T_base_link
        world_T_vicon = pose_stamped_to_matrix(world_vicon)
        odom_T_world = transform_stamped_to_matrix(self.tf_buffer.lookup_transform('odom', 'world', rospy.Time(0)))
        odom_T_base_link = odom_T_world @ world_T_vicon @ self.vicon_T_base

        odom_base = PoseStamped()
        odom_base.header.frame_id = "odom"
        odom_base.pose = matrix_to_pose(odom_T_base_link)
        return odom_base


    def _handle_waypoint_enqueue_srv(self, req):
        self.waypoint_queue_mutex.acquire()
        self.waypoint_queue += [self.transform_waypoint_target(world_vicon_waypoint) for world_vicon_waypoint in req.poses]
        if len(self.waypoint_queue) > 0:
            self.current_waypoint = self.waypoint_queue[0]
        self.waypoint_queue_mutex.release()
        return WaypointEnqueueServiceResponse(success=True, message="Waypoint Enqueue Processed successfully")


    def _handle_waypoint_clear_srv(self, req):
        self.waypoint_queue_mutex.acquire()
        self.waypoint_queue = list()
        self.waypoint_queue_mutex.release()
        return EmptyResponse()


    def _handle_takeoff_srv(self, req):
        # specified as a pose representing world_T_vicon
        take_off_pose = PoseStamped()
        take_off_pose.pose.position.x = 0
        take_off_pose.pose.position.y = 0
        # take_off_pose.pose.position.z = 1.5
        take_off_pose.pose.position.z = 0.5
        take_off_pose.pose.orientation.x = 0
        take_off_pose.pose.orientation.y = 0
        take_off_pose.pose.orientation.z = 0
        take_off_pose.pose.orientation.w = 1.0

        # now as a pose representing odom_T_base
        take_off_pose_transformed = self.transform_waypoint_target(take_off_pose)

        # must start streaming waypoints before entering OFFBOARD mode
        for i in range(100):
            if(rospy.is_shutdown()):
                break
            self.position_pub.publish(take_off_pose_transformed)
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

        rospy.loginfo("Taking off drone.")
        self.current_waypoint = take_off_pose_transformed
        self.has_taken_off = True

        rospy.loginfo("Take off pose reached.")
        return EmptyResponse()
    

    def _handle_land_srv(self, req):
        land_pose = PoseStamped()
        land_pose.header.frame_id = "odom"
        land_pose.pose.position.x = self.current_pose.pose.position.x
        land_pose.pose.position.y = self.current_pose.pose.position.y
        land_pose.pose.position.z = -0.1
        land_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        land_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        land_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        land_pose.pose.orientation.w = self.current_pose.pose.orientation.w

        self.has_taken_off = False
        self.waypoint_queue = list()
        rospy.loginfo("Landing drone.")
        self.current_waypoint = land_pose
        while(not rospy.is_shutdown() or not self.mavros_state.armed):
            if self.current_pose.pose.position.z < self.land_height + 0.2:
                break
            self.position_pub.publish(self.current_waypoint)
            self.rate.sleep()
            
        rospy.loginfo("Drone landed successfully.")  

        return EmptyResponse()


    def _handle_abort_srv(self, req):
        # self.has_taken_off == False
        # set drone mode to STABILIZED
        set_mode = SetModeRequest()
        set_mode.custom_mode = "STABILIZED"
        if self.mavros_set_mode_client.call(set_mode).mode_sent == True:
            rospy.loginfo("STABILIZED enabled.")
        else:
            rospy.logwarn("Failed to set the drone mode to STABILIZED.")

    

    def run(self):
        while(not rospy.is_shutdown()):
            if self.has_taken_off == False:
                self.rate.sleep()
                continue
            
            # target - current
            pos_error, rot_error, pos_diff, rot_diff = pose_error(
                self.current_pose.pose, self.current_waypoint.pose, full_pose=False,
            )
            
            if pos_error < Configs.waypoint_pos_tol and rot_error < Configs.waypoint_rot_tol:
                # rospy.loginfo("Current waypoint reached.") 
                if len(self.waypoint_queue) > 0:
                    self.current_waypoint = self.waypoint_queue.pop(0)
                    rospy.loginfo("Setting the next waypoint.")

            # calculate sub_waypoint as current_pose + delta
            if pos_error > Configs.waypoint_pos_delta:
                pos_diff = pos_diff / pos_error * Configs.waypoint_pos_delta

            if rot_error > Configs.waypoint_rot_delta:
                rot_diff = rot_diff / rot_error * Configs.waypoint_rot_delta
            curr_orientation = self.current_pose.pose.orientation
            curr_pose_quat = [curr_orientation.x, curr_orientation.y, curr_orientation.z, curr_orientation.w]
            rot_diff_quat = quaternion_from_euler(0, 0, rot_diff)
            sub_waypoint_quat = quaternion_multiply(curr_pose_quat, rot_diff_quat)

            current_sub_waypoint = copy.deepcopy(self.current_pose)
            current_sub_waypoint.header.stamp = rospy.Time.now()
            current_sub_waypoint.pose.position.x += pos_diff[0]
            current_sub_waypoint.pose.position.y += pos_diff[1]
            current_sub_waypoint.pose.position.z += pos_diff[2]
            current_sub_waypoint.pose.orientation.x = sub_waypoint_quat[0]
            current_sub_waypoint.pose.orientation.y = sub_waypoint_quat[1]
            current_sub_waypoint.pose.orientation.z = sub_waypoint_quat[2]
            current_sub_waypoint.pose.orientation.w = sub_waypoint_quat[3]

            self.position_pub.publish(current_sub_waypoint)
            self.rate.sleep()
            


if __name__ == "__main__":
    rospy.init_node('waypoint_controller')
    waypoint_controller = WaypointController()
    waypoint_controller.run()
