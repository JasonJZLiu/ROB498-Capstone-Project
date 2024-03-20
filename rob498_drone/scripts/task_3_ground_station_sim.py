#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from configs import Configs

import numpy as np


class Task3GroundStationSim:
    def __init__(self):
        name = 'rob498_drone_' + Configs.team_id

        # wait for the services to become available
        rospy.wait_for_service(name + '/comm/launch')
        rospy.wait_for_service(name + '/comm/test')
        rospy.wait_for_service(name + '/comm/land')
        rospy.wait_for_service(name + '/comm/abort')

        try:
            # create service proxies
            self.srv_launch_client = rospy.ServiceProxy(name + '/comm/launch', Empty)
            self.srv_test_client = rospy.ServiceProxy(name + '/comm/test', Empty)
            self.srv_land_client = rospy.ServiceProxy(name + '/comm/land', Empty)
            self.srv_abort_client = rospy.ServiceProxy(name + '/comm/abort', Empty)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    

    def run(self):
        while (not rospy.is_shutdown()):
            print("1: Launch | 2: Test | 3: Land | 4: Abort")

            key = input()
            print(key)
            if key == '1':
                self.srv_launch_client()
                print("Launch request sent successfully.")
            elif key == '2':
                self.srv_test_client()
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
