#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray

from configs import Configs




class CameraTest:
    def __init__(self):
        pass
      


if __name__ == '__main__':
    rospy.init_node("camera_test")
    camera_test = CameraTest()
    rospy.spin()





    
