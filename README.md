# University of Toronto Engineering Science Robotics Capstone
## Project: Safe Drone Teleoperation with Real Time Mapping and Obstacle Avoidance 
Jack Jiao, Jamie Zhang, Jason Liu, Maxim Radulovic, Rayan Hossain

<img src="https://github.com/JasonJZLiu/ROB498_Capstone/assets/34286328/f1101a9f-74a0-483b-9881-820a2edf474d" width="805"/>

<img src="https://github.com/JasonJZLiu/ROB498_Capstone/assets/34286328/81ec932b-2b7d-4c47-adf8-efb8a0404261" width="400"/>
<img src="https://github.com/JasonJZLiu/ROB498_Capstone/assets/34286328/e6133dd6-4254-423e-9e1f-5c4b39b1f420" width="400"/>

### Real World Video Demos:
- https://www.youtube.com/watch?v=E0sc3qmLEyM
- https://www.youtube.com/watch?v=AxJhX1aww0Q

### Simulation Demos:
- https://www.youtube.com/watch?v=eGf0OMN8-Ao
- https://www.youtube.com/watch?v=Qy-aSO5EVtM
- https://www.youtube.com/watch?v=okVgQpjrboQ

## Project Description:
Teleoperating a drone in a safe manner can be challenging, particularly in cluttered indoor environments with an abundance of obstacles. We present a safe teleoperation system for drones by performing automatic real-time dynamic obstacle avoidance, allowing us to expose a suite of simplified high-level control primitives to the drone operator such as "fly forward", "fly to the left", "fly up", "rotate", etc. This system reduces the complexity and the extent of the manual controls required from drone operators to fly the drone safely. The system accomplishes this by constructing a dynamic map of its environment in real-time and continuously performing path-planning using the map in order to execute a collision-free path to the desired user-specified position target. The main components of the system include the following:

### Localization: 
- Hardware level visual-inertia odometry (VIO) from an Intel RealSense T265 camera
- IMU pose estimates from the flight controller
- Odometry estimates fused by an extended Kalman filter (EKF)

### Mapping: 
- Voxelizes point clouds from an Intel RealSense D435 camera using OctoMap, running onboard on a Jetson Nano at 2 Hz
- Converts voxelized point clouds into 3D occupancy grids with an obstacle buffer range of 0.5 meters as well as an artificial ground and ceilling

### Teleoperation Command Manager
- Receives high-level teleoperation commands from the drone operator
- Computes the desired target position based on the teleoperation command
- Runs path-planning to compute a safe path towards the desired target position

### Path-Planning
- Computes the closest safe grid to the desired target position given a 3D occupancy grid via breadth-first search
- Executes A* path-planning from the current position of the drone to the closest safe target grid, running onboard on a Jetson Nano at 1 Hz
- Sends the optimal path as a series of waypoints to the waypoint controller

### Waypoint Controller
- Interpolates between waypoints to yield a series of position targets
- Sends position targets to the flight controller


## Repository Overview:
This repository contains the source code for both the Gazebo simulation and real-world deployment used for this project.

### Running the Simulation:
```bash
bash bash_scripts/tasks/setup_laptop_sim.sh
roslaunch rob498_drone project_teleop_sim.launch
```

### Real-World Deployment:
For the onboard Jetson Nano:
```bash
bash bash_scripts/setup_jetson_project.sh
roslaunch rob498_drone project_teleop.launch
```

For the laptop that sends the teleoperation commands:
```bash
bash bash_scripts/setup_laptop_project.sh JETSONS_IP_ADDRESS
roslaunch rob498_drone project_laptop.launch
```





