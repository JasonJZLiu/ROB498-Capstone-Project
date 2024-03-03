#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from rob498_drone.srv import WaypointEnqueueService




if __name__ == '__main__':
    rospy.init_node("task_2_vicon")

    rospy.wait_for_service("waypoint/enqueue")
    try:
        pose_list_service = rospy.ServiceProxy("waypoint/enqueue", WaypointEnqueueService)

        pose = PoseStamped()
        pose.pose.position.x = -2
        pose.pose.position.y = -2
        pose.pose.position.z = 1

        poses = [pose]


        resp = pose_list_service(poses)
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)




    
    rospy.spin()
