#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from configs import Configs

from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from rob498_drone.srv import Task3TestService, Task3TestServiceRequest

import numpy as np
from tf.transformations import quaternion_from_euler

# WAYPOINT_1 = Pose()
# WAYPOINT_1.position.x = 2.0
# WAYPOINT_1.position.y = 0.0
# WAYPOINT_1.position.z = 1.5
# WAYPOINT_1.orientation.x = 0
# WAYPOINT_1.orientation.y = 0
# WAYPOINT_1.orientation.z = 0.707
# WAYPOINT_1.orientation.w = -0.707

# WAYPOINT_2 = Pose()
# WAYPOINT_2.position.x = 2.0
# WAYPOINT_2.position.y = 2.0
# WAYPOINT_2.position.z = 1.5
# WAYPOINT_2.orientation.x = 0
# WAYPOINT_2.orientation.y = 0
# WAYPOINT_2.orientation.z = 0
# WAYPOINT_2.orientation.w = 1

# WAYPOINT_3 = Pose()
# WAYPOINT_3.position.x = 0
# WAYPOINT_3.position.y = 2.0
# WAYPOINT_3.position.z = 1.5
# WAYPOINT_3.orientation.x = 0
# WAYPOINT_3.orientation.y = 0
# WAYPOINT_3.orientation.z = 0
# WAYPOINT_3.orientation.w = 1

# WAYPOINT_4 = Pose()
# WAYPOINT_4.position.x = 0
# WAYPOINT_4.position.y = 0
# WAYPOINT_4.position.z = 1.5
# WAYPOINT_4.orientation.x = 0
# WAYPOINT_4.orientation.y = 0
# WAYPOINT_4.orientation.z = 0
# WAYPOINT_4.orientation.w = 1


# WAYPOINT_POSES = PoseArray()
# WAYPOINT_POSES.poses.append(WAYPOINT_1)
# # WAYPOINT_POSES.poses.append(WAYPOINT_2)
# # WAYPOINT_POSES.poses.append(WAYPOINT_3)
# # WAYPOINT_POSES.poses.append(WAYPOINT_4)




class Task3GroundStationSim:
    def __init__(self):
        # wait for the services to become available
        name = 'rob498_drone_' + Configs.team_id
        rospy.wait_for_service(name + '/comm/launch')
        rospy.wait_for_service(name + '/comm/test')
        rospy.wait_for_service(name + '/comm/land')
        rospy.wait_for_service(name + '/comm/abort')

        try:
            # create service proxies
            self.srv_launch_client = rospy.ServiceProxy(name + '/comm/launch', Empty)
            self.srv_test_client = rospy.ServiceProxy(name + '/comm/test', Task3TestService)
            self.srv_land_client = rospy.ServiceProxy(name + '/comm/land', Empty)
            self.srv_abort_client = rospy.ServiceProxy(name + '/comm/abort', Empty)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    

    def generate_waypoints(self, num_waypoints=4):
        waypoint_pose_arr = PoseArray()
        for i in range(num_waypoints):
            # x, y, z, yaw
            drone_pose_min = np.array([-3, -3, 0.5, -np.pi])
            drone_pose_max = np.array([ 3,  3, 2.5,  np.pi])
            drone_pose = np.random.uniform(drone_pose_min, drone_pose_max)
            drone_quat = quaternion_from_euler(0, 0, drone_pose[3])
        
            waypoint = Pose()
            waypoint.position.x = drone_pose[0]
            waypoint.position.y = drone_pose[1]
            waypoint.position.z = drone_pose[2]
            waypoint.orientation.x = drone_quat[0]
            waypoint.orientation.y = drone_quat[1]
            waypoint.orientation.z = drone_quat[2]
            waypoint.orientation.w = drone_quat[3]

            waypoint_pose_arr.poses.append(waypoint)
        
        return waypoint_pose_arr


    def run(self):
        while (not rospy.is_shutdown()):
            print("1: Launch | 2: Test | 3: Land | 4: Abort")

            key = input()
            print(key)
            if key == '1':
                self.srv_launch_client()
                print("Launch request sent successfully.")
            elif key == '2':
                try:
                    world_vicon = rospy.wait_for_message("vicon/ROB498_Drone/ROB498_Drone", TransformStamped)
                except rospy.ROSException as e:
                    rospy.logerr("Timeout waiting for vicon/ROB498_Drone/ROB498_Drone.")
                waypoint_pose_arr = self.generate_waypoints(num_waypoints=4)
                self.srv_test_client(waypoint_pose_arr, world_vicon)
                print("Test request sent successfully.")
            elif key == '3':
                self.srv_land_client()
                print("Land request sent successfully.")
            elif key == '4':
                self.srv_abort_client()
                print("Abort request sent successfully.")
            else:
                print("Invalid input")


if __name__ == "__main__":
    rospy.init_node('task_3_ground_station_sim')
    task_3_ground_station_sim = Task3GroundStationSim()
    task_3_ground_station_sim.run()
    rospy.spin()
