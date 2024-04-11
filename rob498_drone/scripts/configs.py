#! /usr/bin/env python3

class Configs:
    team_id = "02"
    drone_base_link_name = "iris_depth_camera::iris::base_link"
    
    waypoint_pos_tol = 0.2 #0.05
    waypoint_rot_tol = 0.2 #0.05

    waypoint_pos_delta = 0.4
    waypoint_rot_delta = 0.1

    path_planner_max_height = 2.0