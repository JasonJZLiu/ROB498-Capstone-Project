#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, TransformStamped
# from offboard_py.scripts.utils import pose_to_transform_stamped

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Quaternion, Twist, PoseArray, Transform

from nav_msgs.msg import Odometry
import tf2_ros, time



def pose_to_transform_stamped(pose: Pose, frame_id="parent_frame", child_frame_id="child_frame"):
    # Extract position and orientation from the Pose message
    position = pose.position
    orientation = pose.orientation

    # Create a TransformStamped message
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = frame_id
    transform_stamped.child_frame_id = child_frame_id

    transform_stamped.transform.translation.x = position.x
    transform_stamped.transform.translation.y = position.y
    transform_stamped.transform.translation.z = position.z

    transform_stamped.transform.rotation.x = orientation.x
    transform_stamped.transform.rotation.y = orientation.y
    transform_stamped.transform.rotation.z = orientation.z
    transform_stamped.transform.rotation.w = orientation.w

    return transform_stamped


def transform_stamped_to_odometry(transform_stamped: TransformStamped):
    # Create an Odometry message
    odom = Odometry()
    
    # Copy header information
    odom.header = transform_stamped.header
    
    # Set child frame ID (e.g., "base_link")
    odom.child_frame_id = "base_link"
    
    # Copy pose information from TransformStamped to Odometry
    odom.pose.pose.position.x = transform_stamped.transform.translation.x
    odom.pose.pose.position.y = transform_stamped.transform.translation.y
    odom.pose.pose.position.z = transform_stamped.transform.translation.z
    odom.pose.pose.orientation.x = transform_stamped.transform.rotation.x
    odom.pose.pose.orientation.y = transform_stamped.transform.rotation.y
    odom.pose.pose.orientation.z = transform_stamped.transform.rotation.z
    odom.pose.pose.orientation.w = transform_stamped.transform.rotation.w
    
    # Set pose covariance to zero (unknown)
    odom.pose.covariance = [0] * 36
    
    # Set linear and angular velocities to zero (unknown)
    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.y = 0
    odom.twist.twist.angular.z = 0
    
    # Set twist covariance to zero (unknown)
    odom.twist.covariance = [0] * 36
    
    return odom


class GazeboLinkPose:
  link_name = ''
  link_pose = TransformStamped()
  def __init__(self, link_name):
    self.link_name = link_name
    self.link_name_rectified = "vicon_pose" #link_name.replace("::", "_")

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")


    # wait until gazebo is fully loaded
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        # Gazebo is ready, proceed with your operations
    except rospy.ROSException as e:
        rospy.logerr("Timeout waiting for /gazebo/set_model_state service.")
        # Handle timeout or failure to find the service


    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)

    self.pose_pub = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size = 10)
    self.br = tf2_ros.TransformBroadcaster()

    self.mavros_vision_pose_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)

  def callback(self, data):
    try:
        ind = data.name.index(self.link_name)

        print(data.pose[ind])
        self.link_pose = pose_to_transform_stamped(data.pose[ind])
    except ValueError:
        print(f"Invalid link name {self.link_name}, choose one from: {data.name}")

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose("iris_depth_camera::iris::base_link")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      gp.pose_pub.publish(gp.link_pose)
      gp.br.sendTransform(gp.link_pose)

      odom = transform_stamped_to_odometry(gp.link_pose)
      odom.header.frame_id = "odom"
      odom.child_frame_id = "base_link"
      odom.header.stamp=rospy.Time.now()
      gp.mavros_vision_pose_pub.publish(odom)

      rate.sleep()

  except rospy.ROSInterruptException:
    pass