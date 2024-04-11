#!/usr/bin/env python3

import heapq
import numpy as np


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
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
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

    # Return None if there is no path
    return None  



def find_closest_free_grid(grid, target_index):
        # runs BFS
        delta = 1
        directions = np.array([
             [-delta, 0, 0],
            [0, delta, 0], [0, -delta, 0],
            [0, 0, delta], [0, 0, -delta], [delta, 0, 0],
        ])

        # ensure the target is within the grid
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