#!/usr/bin/env python3

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from utils import *


class WorldToOdom:
    def __init__(self):
        pass

    

if __name__ == '__main__':
    rospy.init_node('world_to_odom')
    vicon_sim = WorldToOdom()
    rospy.spin()
