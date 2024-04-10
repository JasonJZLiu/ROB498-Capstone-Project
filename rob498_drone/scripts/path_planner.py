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



# from queue import PriorityQueue

# class Node:
#     def __init__(self, position: tuple, parent: tuple):
#         self.position = position
#         self.parent = parent
#         self.g = np.inf
#         self.h = np.inf
#         self.f = np.inf

#     def __lt__(self, other):
#         return self.f < other.f
    

# def a_star(grid, start, goal):
#     # Create start and goal nodes
#     start_node = Node(start, None)
#     goal_node = Node(goal, None)
    
#     # Initialize both open and closed list
#     open_list = PriorityQueue()
#     open_list.put((start_node.f, start_node))
#     closed_list = set()
    
#     # Loop until you find the end
#     while not open_list.empty():
#         # Get the current node
#         current_f, current_node = open_list.get()
#         closed_list.add(current_node.position)
        
#         # Found the goal
#         if current_node.position == goal_node.position:
#             path = []
#             while current_node is not None:
#                 path.append(current_node.position)
#                 current_node = current_node.parent
#             # Return reversed path
#             return path[::-1]  
        
#         # Generate children
#         for new_position in [(0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0), (0, 0, 1), (0, 0, -1),  # Adjacent cells
#                              (-1, -1, 0), (-1, 1, 0), (1, -1, 0), (1, 1, 0),  # Diagonals on the same level
#                              (0, -1, 1), (0, 1, 1), (-1, 0, 1), (1, 0, 1), (0, -1, -1), (0, 1, -1), (-1, 0, -1), (1, 0, -1),  # Vertical movements
#                              (-1, -1, 1), (-1, 1, 1), (1, -1, 1), (1, 1, 1), (-1, -1, -1), (-1, 1, -1), (1, -1, -1), (1, 1, -1)]:  # Diagonal movements in 3D
#             node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2] + new_position[2])
            
#             # Make sure within range
#             if node_position not in grid:
#                 continue
            
#             # Create new node
#             new_node = Node(node_position, current_node)
            
#             # Child is on the closed list
#             if new_node.position in closed_list:
#                 continue
            
#             # Create the f, g, and h values
#             new_node.g = current_node.g + np.linalg.norm(np.array(node_position) - np.array(current_node.position))
#             new_node.h = np.linalg.norm(np.array(goal_node.position) - np.array(new_node.position))
#             new_node.f = new_node.g + new_node.h
            
#             # Child is already in the open list
#             for open_node_f, open_node in open_list.queue:
#                 if new_node == open_node and new_node.g > open_node.g:
#                     continue
            
#             # Add the child to the open list
#             open_list.put((new_node.f, new_node))

#      # Path not found
#     return None 



import heapq
def heuristic(a, b):
    """
    Compute the Manhattan distance between two points in a 3D grid.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def get_neighbors(node, grid):
    """
    Generate all possible neighbors for a given node in a 3D grid,
    excluding those with obstacles or outside the grid boundaries.
    """
    neighbors = []
    for dz in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dz == 0 and dy == 0 and dx == 0:
                    continue  # Skip the current node itself
                new_x, new_y, new_z = node[0] + dx, node[1] + dy, node[2] + dz
                if 0 <= new_x < grid.shape[0] and 0 <= new_y < grid.shape[1] and 0 <= new_z < grid.shape[2]:
                    if grid[new_x, new_y, new_z] == 0:  # No obstacle
                        neighbors.append((new_x, new_y, new_z))
    return neighbors


def a_star(grid, start, goal):
    """
    Perform A* search to find a path from start to goal in a 3D grid.
    """
    start, goal = tuple(start), tuple(goal)  # Convert to tuples
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current_g, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Reverse the path

        for neighbor in get_neighbors(current, grid):
            tentative_g_score = g_score[current] + 1  # Assuming cost = 1 for any move
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                if neighbor not in [i[2] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], g_score[neighbor], neighbor))

    return None  # Return an empty path if there is no path



class PathPlanner:
    def __init__(self):
        self.resolution = 0.2
        rospy.wait_for_service("waypoint/enqueue")
        self.waypoint_enqueue_client = rospy.ServiceProxy("waypoint/enqueue", WaypointEnqueueService)

        rospy.wait_for_service("waypoint/clear")
        self.waypoint_clear_client = rospy.ServiceProxy("waypoint/clear", Empty)


        self.obstacle_pc_sub = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback=self._obstacle_pointcloud_callback)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self._pose_callback)

        rospy.Service("path_planner/run_astar", AStarService, self._handle_astar_srv)
        print("PathPlanner: Finished setting up AStarService.")

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
        


        target_idx_adjusted = target_idx - min_indices

        # get a free grid idx closest to target_point
        safe_target_idx_adjusted = self.find_closest_free_grid(grid, target_idx_adjusted)
        if safe_target_idx_adjusted is None:
            print("CANNOT FIND A VALID SAFE TARGET")
            return AStarServiceResponse(success=False, message="CANNOT FIND A VALID SAFE TARGET")


        safe_target_pos = self.adjusted_idx_to_odom_pos(safe_target_idx_adjusted, min_indices)
        self.visualize_safe_target_point(safe_target_pos)

        # starting spot for a star
        starting_idx_adjusted = self.drone_position_idx - min_indices


        adjusted_idx_path = a_star(grid, starting_idx_adjusted, safe_target_idx_adjusted)
        if adjusted_idx_path is None:
            print("CANNOT FIND A VALID PATH")
            return AStarServiceResponse(success=False, message="CANNOT FIND A VALID PATH") 
        

        waypoint_list = list()

        waypoint_path = Path()
        waypoint_path.header.frame_id = "world"  # Or the appropriate frame ID for your application
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
            

        self.waypoint_path_publisher.publish(waypoint_path)

        self.waypoint_clear_client()
        self.waypoint_enqueue_client(waypoint_list)


 
        return AStarServiceResponse(success=True, message="A* Service Processed successfully")




    def find_closest_free_grid(self, grid, target_index):
        # Runs BFS

        detla = 2
        directions = np.array([
             [-detla, 0, 0],
            [0, detla, 0], [0, -detla, 0],
            [0, 0, detla], [0, 0, -detla], [detla, 0, 0],
        ])

        target_index = np.clip(target_index, [0, 0, 0], np.array(grid.shape) - 1)

        # Check if the start index itself is free
        if grid[tuple(target_index)] == 0:
            return tuple(target_index)

        queue = [target_index]
        visited = set([tuple(target_index)])
        
        while queue:
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





      
if __name__ == '__main__':
    rospy.init_node("path_planner")
    path_planner = PathPlanner()
    rospy.spin()

    





    
