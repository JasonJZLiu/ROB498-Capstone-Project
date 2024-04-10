#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from rob498_drone.srv import AStarService, AStarServiceResponse
from rob498_drone.srv import WaypointEnqueueService

from configs import Configs

import numpy as np













class PathPlanner:
    def __init__(self):
        self.resolution = 0.5
        rospy.wait_for_service("waypoint/enqueue")
        self.waypoint_enqueue_client = rospy.ServiceProxy("waypoint/enqueue", WaypointEnqueueService)


        self.obstacle_pc_sub = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback=self._obstacle_pointcloud_callback)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self._pose_callback)

        rospy.Service("path_planner/run_astar", AStarService, self._handle_astar_srv)
        print("PathPlanner: Finished setting up AStarService.")


    def _obstacle_pointcloud_callback(self, point_cloud_2):
        point_cloud_list = [point for point in pc2.read_points(point_cloud_2, field_names=("x", "y", "z"), skip_nans=True)]

        point_cloud_arr = np.asarray(point_cloud_list)
        point_cloud_arr += -self.resolution/2
        point_cloud_arr /= self.resolution
        self.point_cloud_arr_idx = np.round(point_cloud_arr).astype(int)


        



    def _pose_callback(self, pose_stamped):
        # in frame: map
        current_pos = pose_stamped.pose.position
        drone_position = np.array([current_pos.x, current_pos.y, current_pos.z])
        self.drone_position_idx = self.odom_pos_to_idx(drone_position)


 
    def odom_pos_to_idx(self, pos):
        pos /= self.resolution
        idx = np.round(pos).astype(int)
        return idx



    def _handle_astar_srv(self, req):

        # target_point = [2, 2, 0] odom_pos
        target_point = np.array([req.target_point.x, req.target_point.y, req.target_point.z])
        target_idx = self.odom_pos_to_idx(target_point)
        print(target_idx)

        # construct the grid array used by A*
        obstacle_indices = self.point_cloud_arr_idx
        min_indices = np.min(obstacle_indices, axis=0)
        max_indices = np.max(obstacle_indices, axis=0)

        # Calculate grid size; add 1 because indices are inclusive, and offset for negatives
        grid_size = max_indices - min_indices + 1

        # Initialize grid to zeros; using int datatype for simplicity
        grid = np.zeros(grid_size, dtype=int)

        # Adjust indices to be zero-based for the grid
        adjusted_indices = obstacle_indices - min_indices

        # Mark each obstacle in the grid
        for idx in adjusted_indices:
            grid[tuple(idx)] = 1 
        

        staring_idx_adjusted = self.drone_position_idx - min_indices
        target_idx_adjusted = target_idx - min_indices
        print(min_indices)


        # TODO: get a free grid idx closest to target_point
        safe_target_idx_adjusted = self.find_closest_free_space_list_queue(grid, target_idx_adjusted)
        
        print(safe_target_idx_adjusted + min_indices)
        print("\n")



        return AStarServiceResponse(success=True, message="A* Service Processed successfully")




    def find_closest_free_space_list_queue(self, grid, start_index):
        """
        Finds the closest grid index to 'start_index' that has a value of 0 (free space), using a list for the queue
        and treating 'start_index' as a numpy array for easier arithmetic operations.

        :param grid: 3D numpy array representing the grid.
        :param start_index: Numpy array representing the start index, adjusted to grid's indexing.
        :return: Tuple representing the closest free space index, adjusted to grid's indexing, or None if not found.
        """
        directions = np.array([
             [-1, 0, 0],
            [0, 1, 0], [0, -1, 0],
            [0, 0, 1], [0, 0, -1], [1, 0, 0],
        ])
        print("pre clipped", start_index)

        start_index = np.clip(start_index, [0, 0, 0], np.array(grid.shape) - 1)
        print("clipped", start_index)

        # Check if the start index itself is free
        if grid[tuple(start_index)] == 0:
            print("WTF")
            return tuple(start_index)

        queue = [start_index]
        visited = set([tuple(start_index)])
        
        while queue:
            print("BFS STARTED")
            current = queue.pop(0)
            current_np = np.array(current)
            
            for d in directions:
                neighbor = current_np + d
                neighbor_tuple = tuple(neighbor)
                
                # Check boundaries
                if (0 <= neighbor[0] < grid.shape[0]) and (0 <= neighbor[1] < grid.shape[1]) and (0 <= neighbor[2] < grid.shape[2]):
                    if neighbor_tuple not in visited:
                        visited.add(neighbor_tuple)
                        
                        # Check if the neighbor is free space
                        if grid[neighbor_tuple] == 0:
                            return neighbor
                        
                        queue.append(neighbor)
                        
        # If no free space is found
        return None







    

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




      
if __name__ == '__main__':
    rospy.init_node("path_planner")
    path_planner = PathPlanner()
    rospy.spin()

    





    
