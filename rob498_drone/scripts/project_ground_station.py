#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from configs import Configs

from geometry_msgs.msg import PoseArray, Pose, TransformStamped, Point
from rob498_drone.srv import ProjectTeleopService, ProjectTeleopServiceRequest




class ProjectGroundStation:
    def __init__(self):
        name = 'rob498_drone_' + Configs.team_id

        # wait for the services to become available
        rospy.wait_for_service(name + '/comm/launch')
        rospy.wait_for_service(name + '/comm/land')
        rospy.wait_for_service(name + '/comm/abort')
        rospy.wait_for_service(name + '/comm/teleop')

        try:
            # create service proxies
            self.srv_launch_client = rospy.ServiceProxy(name + '/comm/launch', Empty)
            self.srv_land_client = rospy.ServiceProxy(name + '/comm/land', Empty)
            self.srv_abort_client = rospy.ServiceProxy(name + '/comm/abort', Empty)
            self.srv_teleop_client = rospy.ServiceProxy(name + '/comm/teleop', ProjectTeleopService)

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    

    def run(self):
        while (not rospy.is_shutdown()):
            print("1: Launch | 2: NULL | 3: Land | 4: Abort | WASD for Teleop")

            key = input()
            print(key)
            if key == '1':
                self.srv_launch_client()
                print("Launch request sent successfully.")
                print("Test request sent successfully.")
            elif key == '3':
                self.srv_land_client()
                print("Land request sent successfully.")
            elif key == '4':
                self.srv_abort_client()
                print("Abort request sent successfully.")
            elif key in ["w", "s", "a", "d", "q", "e"]:
                self.srv_teleop_client(key)
                print("Teleop request sent successfully.")
            else:
                print("Invalid input")
    




if __name__ == "__main__":
    rospy.init_node('task_3_ground_station_sim')
    project_ground_station_sim = ProjectGroundStation()
    project_ground_station_sim.run()
    rospy.spin()
