#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Point, Pose
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from rob498_drone.srv import AStarService, AStarServiceResponse
from rob498_drone.srv import WaypointEnqueueService
from std_srvs.srv import Empty, EmptyResponse

from configs import Configs
import numpy as np

from a_star import a_star, find_closest_free_grid


class PathPlanner:
    def __init__(self):
        # ensure this resolution is the same as the one used in octomap
        self.resolution = 0.5

        max_height = float(Configs.path_planner_max_height)
        self.max_height_idx = self.odom_pos_to_idx(np.array([0, 0, max_height]))[2]

        rospy.wait_for_service("waypoint/enqueue")
        self.waypoint_enqueue_client = rospy.ServiceProxy("waypoint/enqueue", WaypointEnqueueService)

        rospy.wait_for_service("waypoint/clear")
        self.waypoint_clear_client = rospy.ServiceProxy("waypoint/clear", Empty)

        rospy.wait_for_message("/octomap_point_cloud_centers", PointCloud2)
        self.obstacle_pc_sub = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback=self._obstacle_pointcloud_callback)

        rospy.wait_for_message("mavros/local_position/pose", PoseStamped)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self._pose_callback)

        rospy.Service("path_planner/run_astar", AStarService, self._handle_astar_srv)


        self.safe_target_point_marker_pub = rospy.Publisher('/project_safe_target_point_marker', Marker, queue_size=10)
        self.waypoint_path_publisher = rospy.Publisher('/project_waypoint_path', Path, queue_size=10)


    def _obstacle_pointcloud_callback(self, point_cloud_2):
        point_cloud_list = [point for point in pc2.read_points(point_cloud_2, field_names=("x", "y", "z"), skip_nans=True)]

        point_cloud_arr = np.asarray(point_cloud_list)
        point_cloud_arr += -self.resolution/2
        point_cloud_arr /= self.resolution
        self.point_cloud_arr_idx = np.round(point_cloud_arr).astype(int)


    def _pose_callback(self, pose_stamped):
        # in frame: map
        self.drone_pose = pose_stamped.pose
        current_pos = pose_stamped.pose.position
        drone_position = np.array([current_pos.x, current_pos.y, current_pos.z])
        self.drone_position_idx = self.odom_pos_to_idx(drone_position)



    def _handle_astar_srv(self, req):

        target_point = np.array([req.target_point.x, req.target_point.y, req.target_point.z])
        target_idx = self.odom_pos_to_idx(target_point)

        # construct the grid array used by A*
        obstacle_indices = self.point_cloud_arr_idx

        # self.odom_pos_to_idx()

        obstacle_idx_with_curr_pos = np.vstack((obstacle_indices, target_idx, self.drone_position_idx))
        min_indices = np.min(obstacle_idx_with_curr_pos, axis=0) - 2
        max_indices = np.max(obstacle_idx_with_curr_pos, axis=0) + 2

        # Calculate grid size; add 1 because indices are inclusive, and offset for negatives
        grid_size = max_indices - min_indices + 1

        # Initialize grid to zeros; using int datatype for simplicity
        grid = np.zeros(grid_size, dtype=int)

        # Adjust indices to be zero-based for the grid
        adjusted_indices = obstacle_indices - min_indices

        # Mark each obstacle in the grid
        for idx in adjusted_indices:
            for offset in [[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]:
                        #    [2, 0, 0], [-2, 0, 0], [0, 2, 0], [0, -2, 0], [0, 0, 2], [0, 0, -2]]:
                grid[tuple(idx + np.asarray(offset))] = 1


        max_height_idx_adjusted = self.max_height_idx - min_indices[2]

        if max_height_idx_adjusted < grid.shape[2]:
            grid[:, :, max_height_idx_adjusted:] = 1
        

        target_idx_adjusted = target_idx - min_indices

        # get a free grid idx closest to target_point
        safe_target_idx_adjusted = find_closest_free_grid(grid, target_idx_adjusted)
        if safe_target_idx_adjusted is None:
            rospy.loginfo("PathPlanner: Cannot find a valid safe target!")
            self.waypoint_clear_client()
            self.clear_path_visualization()
            return AStarServiceResponse(success=False, message="PathPlanner: Cannot find a valid safe target!")


        safe_target_pos = self.adjusted_idx_to_odom_pos(safe_target_idx_adjusted, min_indices)
        self.visualize_safe_target_point(safe_target_pos)


        starting_idx_adjusted = self.drone_position_idx - min_indices
        adjusted_idx_path = a_star(grid, starting_idx_adjusted, safe_target_idx_adjusted)
        
        if adjusted_idx_path is None:
            rospy.loginfo("PathPlanner: Cannot find a valid path!")
            self.waypoint_clear_client()
            self.clear_path_visualization()
            return AStarServiceResponse(success=False, message="PathPlanner: Cannot find a valid path!") 
        else:
            rospy.loginfo("PathPlanner: Found a path!")
        

        waypoint_list = list()
        waypoint_path = Path()
        waypoint_path.header.frame_id = "world"
        waypoint_path.header.stamp = rospy.Time.now()

        for i, adjusted_idx_waypoint in enumerate(adjusted_idx_path):
            waypoint_pos = self.adjusted_idx_to_odom_pos(adjusted_idx_waypoint, min_indices)
            waypoint_pose = Pose()
            waypoint_pose.position.x = waypoint_pos[0]
            waypoint_pose.position.y = waypoint_pos[1]
            waypoint_pose.position.z = waypoint_pos[2]
            waypoint_pose.orientation = self.drone_pose.orientation

            waypoint_pose_stamped = PoseStamped()
            waypoint_pose_stamped.header.stamp = rospy.Time.now()
            waypoint_pose_stamped.header.seq = i
            waypoint_pose_stamped.header.frame_id = "vicon"
            waypoint_pose_stamped.pose = waypoint_pose

            waypoint_list.append(waypoint_pose_stamped)
            waypoint_path.poses.append(waypoint_pose_stamped)
            

        self.waypoint_clear_client()
        self.waypoint_enqueue_client(waypoint_list)

        # for visualization
        self.waypoint_path_publisher.publish(waypoint_path)

        return AStarServiceResponse(success=True, message="PathPlanner A* Service Processed Successfully!")



    def odom_pos_to_idx(self, pos):
        pos /= self.resolution
        idx = np.round(pos).astype(int)
        return idx


    def adjusted_idx_to_odom_pos(self, adjusted_idx, min_indices):
        # TODO: make this vectorized
        idx = (adjusted_idx + min_indices).astype(float)
        odom_pos = idx * self.resolution + self.resolution/2
        return odom_pos



    def visualize_occupancy_grid(self, grid, min_indices):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        # Extract obstacle coordinates for visualization
        obstacle_coords = np.argwhere(grid == 1)

        # Adjust coordinates to account for the original indices (add min_indices back)
        obstacle_coords_adjusted = obstacle_coords + min_indices

        # Plotting
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Scatter plot for obstacles
        ax.scatter(obstacle_coords_adjusted[:,0], obstacle_coords_adjusted[:,1], obstacle_coords_adjusted[:,2], c='red', marker='o')

        # Setting labels and title
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        ax.set_title('3D Grid Visualization')

        # Show the plot
        plt.show()
    


    def visualize_safe_target_point(self, target_point):
        # visualization
        marker = Marker()
        marker.header.frame_id = "world"  # Adjust based on your TF frames
        marker.header.stamp = rospy.Time.now()
        marker.ns = "spheres"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target_point[0]
        marker.pose.position.y = target_point[1]
        marker.pose.position.z = target_point[2]
        marker.pose.orientation.w = 1.0
        
        # Size of the sphere
        sphere_diameter = 0.2  # Adjust the size of the sphere here
        marker.scale.x = sphere_diameter  # Diameter of the sphere in X direction
        marker.scale.y = sphere_diameter  # Diameter of the sphere in Y direction
        marker.scale.z = sphere_diameter  # Diameter of the sphere in Z direction
        
        # Color and transparency
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.safe_target_point_marker_pub.publish(marker)
    
    
    def clear_path_visualization(self):
        empty_path = Path()
        empty_path.header.frame_id = "world"
        empty_path.header.stamp = rospy.Time.now()
        self.waypoint_path_publisher.publish(empty_path)



      
if __name__ == '__main__':
    rospy.init_node("path_planner")
    path_planner = PathPlanner()
    rospy.spin()

    





    
