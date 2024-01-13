# ROS Navigation Project

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

This repository contains a ROS (Robot Operating System) project showcasing a simple navigation system with three nodes. The nodes include a goal handler, a service for retrieving the last target, and a service for providing information about the current position and velocity.

## Nodes

1. **set_target_client:**  
   - Handles user input to set new goals or cancel the current goal.
   - Publishes position and velocity information.
   - Utilizes an action client to send goals to the "reaching_goal" action server.
     
![flow chart](https://github.com/Pezhman-Rezaei/RT_Assignment_2/assets/150551888/02fd086a-d1c0-43d9-b37a-3a7ec9351424)

2. **last_target_service:**  
   - Provides a service to retrieve the last desired x and y positions set by the user.

3. **info_service:**  
   - Provides a service to get information about the current position and velocity.
   - Subscribes to the "/pos_vel" topic for continuous monitoring.
   - Logs information every 0.5 seconds.

## Setup and Dependencies

1. Install ROS Kinetic on your system.
2. Clone this repository:

    ```bash
    git clone https://github.com/your-username/ros-navigation-project.git
    ```

3. Navigate to the project directory and build the ROS packages:

    ```bash
    cd ros-navigation-project
    catkin_make
    source devel/setup.bash
    ```

## Usage

- Run the ROS nodes:

    ```bash
    rosrun your_package_name set_target_client.py
    ```

- Open additional terminals and run other nodes as needed.

## Configuration

- Update parameters in the launch files or directly in the ROS parameter server as needed.
- Modify the source code for specific requirements.

## Continuous Monitoring

The `info_service` node continuously monitors position and velocity, logging information every 0.5 seconds. Adjust the duration as needed in the source code.

## Contributing

Feel free to contribute to this project by opening issues or submitting pull requests. Your feedback and contributions are welcome!
