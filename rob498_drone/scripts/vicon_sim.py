#!/usr/bin/env python3

import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped

from configs import Configs
from utils import pose_to_transform_stamped


class ViconSim:
    def __init__(self, link_name):
        self.link_name = link_name
        self.link_transform_stamped = TransformStamped()
    
        # wait until gazebo is fully loaded
        try:
            rospy.wait_for_service('/gazebo/set_model_state')
        except rospy.ROSException as e:
            rospy.logerr("Timeout waiting for /gazebo/set_model_state service.")

        rospy.sleep(1.0)

        self.link_state_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self._link_state_sub_callback)
        self.vicon_sim_pub = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size=10)
        self.rate = rospy.Rate(10)


    def _link_state_sub_callback(self, data):
        try:
            link_pose = data.pose[data.name.index(self.link_name)]
            self.link_transform_stamped = pose_to_transform_stamped(
                pose=link_pose,
                frame_id="world",
                child_frame_id="vicon",
            )
        except ValueError:
            print(f"The given link_name {self.link_name} cannot be found in {data.name}.")


    def run(self):
        while not rospy.is_shutdown():
            self.vicon_sim_pub.publish(self.link_transform_stamped)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('vicon_sim')
    link_name = Configs.drone_base_link_name
    vicon_sim = ViconSim(link_name)
    vicon_sim.run()
