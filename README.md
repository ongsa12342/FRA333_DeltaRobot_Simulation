### Delta Robot Simulation
This repository contains a dynamic simulation of a Delta Robot. The simulation is designed to model the motion and behavior of a Delta Robot in a dynamic environment.

This project is a part of FRA333 Robot Kinematics @ Institute of Field Robotics, King Mongkut’s University of Technology Thonburi
### Description

Forward Kinematics
Determines the end-effector's Cartesian position (x, y, z) and orientation (roll, pitch, yaw) based on joint angles.

Inverse Kinematics
Computes joint angles needed to achieve a specified end-effector position and orientation in the workspace.

Jacobian
Relates joint velocities to end-effector linear and angular velocities, crucial for velocity control and dynamic analysis.

Trajectory Planning
Generates smooth and feasible paths for the Delta Robot's end-effector over time, ensuring efficient movements.

Dynamics
Models forces and torques acting on the robot, considering factors like mass distribution and external forces, providing a realistic representation of the Delta Robot's behavior in dynamic scenarios.

The integration of forward and inverse kinematics, Jacobian, trajectory planning, and dynamics in the Delta Robot dynamic simulation allows for a comprehensive study of the robot's motion, aiding in the development and optimization of control strategies for diverse applications.

### Table of Contents
* [Project Delta robot](#project-delta-robot)
    - [Installation](#installation)
    - [Usage](#usage)
    - [Configuration](#configuration)
    - [Contributing](#contributing)
    - [License](#license)
    - [Acknowledgements](#acknowledgements)
    - [Forward Kinematics](#forward-kinematics)
    - [Inverse Kinematics](#inverse-kinematics)
    - [Jacobian](#jacobian)
    - [Trajectory](#trajectory)
    - [MoveJ](#movej)
    - [MoveL](#movel)
    - [Dynamic](#dynamic)

### Installation
Clone the repository and install the dependencies:

git clone 
### Features
--
### Usage
--
### Run the Delta Robot
--
### Configuration 
robot:
  dimensions:
    arm_length: 100
    base_radius: 50
  speed: 5
### Technologies Used
--
### Setup
--
### Acknowledgements
This project is part of the coursework for FRA333 Robot Kinematics at the Institute of Field Robotics, King Mongkut’s University of Technology Thonburi. Special thanks to the course instructors for their guidance and support.

Feel free to explore, modify, and extend this project for educational and research purposes.
### Contact
--
# References
