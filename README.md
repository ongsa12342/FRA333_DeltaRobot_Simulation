=======
# Delta Robot Simulation
This repository contains a dynamic simulation of a Delta Robot. The simulation is designed to model the motion and behavior of a Delta Robot in a dynamic environment.

This project is a part of FRA333 Robot Kinematics @ Institute of Field Robotics, King Mongkut’s University of Technology Thonburi

## Delta Robot Demo

https://github.com/ongsa12342/FRA333_DeltaRobot_Simulation/assets/122738446/f0668d2d-a4bf-401e-acf1-20e3180f423f

## Description

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

## Table of Contents
* [Delta Robot Simulation](#Delta-Robot-Simulation)
    - [Delta Robot Demo](#Delta-Robot-Demo)
    - [Description](#Description)
    - [Prerequisites](#Prerequisites)
    - [Installation](#installation)
    - [Usage](#usage)
    - [Configuration](#configuration)
    - [Medtodology](#Medtodology)
      - [Forward Kinematics](#forward-kinematics)
      - [Inverse Kinematics](#inverse-kinematics)
      - [Jacobian](#jacobian)
      - [Dynamic](#dynamic)
    - [Acknowledgements](#acknowledgements)
    - [References](#References)

# Getting Started

## Prerequisites

Ensure you have the following dependencies installed:

- `vpython`
- `numpy`


## Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/ongsa12342/FRA333_DeltaRobot_Simulation
pip install vpython
pip install numpy
```

<br>

## Usage

<img width="562" alt="delta_robor_simulation" src="https://github.com/ongsa12342/FRA333_DeltaRobot_Simulation/assets/122738446/d7b2de15-fe3c-4a17-b857-30c4ba7be967">

run demo.py

### 1. Position Input

- **Description:** Input the desired X, Y, and Z coordinates in millimeters. After entering the values, click "Enter," and the robot will move to the specified position using Inverse Kinematics. Following this, the robot will undergo free-fall due to gravity.

### 2. Angle Input

- **Description:** Enter the values for Theta1, Theta2, and Theta3. The robot will adjust its joint positions using Forward Kinematics. Subsequently, the robot will fall under the influence of gravity.

### 3. Destination Input

- **Description:** Input the destination coordinates (X, Y, Z). Upon entering, waypoints will appear, and a trajectory will be generated to visualize the robot's path. Click "Enter," and the robot will move to the specified destination using a controller (P Control Cascade Loop with velocity and position). After, the robot will not fall due to applied torque. Additionally, you can switch between trajectory move types (moveL and moveJ) by clicking the corresponding buttons. The trajectory will re-generate accordingly.
### Configuration 
- robot:

  - dimensions:

    - frame_lenght = 0.693 m
                          
    - endeffector_lenght = 0.156 m
                          
    - upper_link = 0.235 m
                          
    - lower_link = 0.800 m

  - Mass:
    
    - upper_link_mass = 1 kg
    
    - lower_link_mass = 0.5 kg
    
    - endeffector_mass = 2 kg
  
  - robot constraint

    - Max Angular Velocity = 11.453 m/s

    - Max Linear Velocity = 10 m/s

    - Max Angular Acceleration = 174.532 m/s^2

    - Max Linear Acceleration = 100 m/s^2

## Medthodology

### Forward Kinematics

<img width="562" alt="delta_robor_simulation" src="https://github.com/ongsa12342/FRA333_DeltaRobot_Simulation/assets/113016544/cfcc90b5-09e2-4d52-a04b-6e3302a98ccb">

 Find Coordinates of points J'1, J'2, J'3

 <img width="362" alt="delta_robor_simulation" src="https://github.com/ongsa12342/FRA333_DeltaRobot_Simulation/assets/113016544/fc62a7d0-a75e-4360-a651-f885f6a79cd5">

 &emsp;&emsp;&emsp; $OF_{1}\ =\ OF_{2}\ =\ OF_{3}\ =\frac{f\cdot\tan\left(30\right)}{2}\ =\ \frac{f}{2\cdot\sqrt{3}}$

 &emsp;&emsp;&emsp; $J_{1}J_{1}'\ =\ J_{2}J_{2}'\ =\ J_{3}J_{3}'\ =\frac{e\cdot\tan\left(30\right)}{2}\ =\ \frac{e}{2\cdot\sqrt{3}}$

 &emsp;&emsp;&emsp; $F_{1}J_{1}\ =\ r_{f}\cdot\cos\left(\theta_{1}\right),\ \ F_{2}J_{2}\ =\ r_{f}\cdot\cos\left(\theta_{2}\right),\ F_{3}J_{3}\ =\ r_{f}\cdot\cos\left(\theta_{3}\right)\ $

 &emsp;&emsp;&emsp;

 &emsp;&emsp;&emsp; $J'_1: (x_1, y_1, z_1) = \left(0, -\frac{{f - e}}{{2\sqrt{3}}} - r_f \cos(\theta_1), -r_f \sin(\theta_1)\right)\$

  &emsp;&emsp;&emsp; $J'_2: (x_2, y_2, z_2) = \left(\left[\frac{{f - e}}{{2\sqrt{3}}} + r_f \cos(\theta_2)\right] \cos(30^\circ), \left[\frac{{f - e}}{{2\sqrt{3}}} - r_f \cos(\theta_2)\right] \sin(30^\circ), -r_f \sin(\theta_2)\right)\$

  &emsp;&emsp;&emsp; $J'_3: (x_3, y_3, z_3) = \left(\left[\frac{{f - e}}{{2\sqrt{3}}} + r_f \cos(\theta_3)\right] \cos(30^\circ), \left[\frac{{f - e}}{{2\sqrt{3}}} - r_f \cos(\theta_3)\right] \sin(30^\circ), -r_f \sin(\theta_3)\right)\$

  Equation of three spheres

  &emsp;&emsp;&emsp; $x^{2}\ +\ \left(y-y_{1}\right)^{2}\ +\ \left(z-z_{1}\right)^{2}\ =\ r_{e}^{2}$

  &emsp;&emsp;&emsp; $\left(x-x_{2}^{ }\right)^{2}\ +\ \left(y-y_{2}\right)^{2}\ +\ \left(z-z_{2}\right)^{2}\ =\ r_{e}^{2}$

  &emsp;&emsp;&emsp; $\left(x-x_{3}^{ }\right)^{2}\ +\ \left(y-y_{3}\right)^{2}\ +\ \left(z-z_{3}\right)^{2}\ =\ r_{e}^{2}$

  &emsp;&emsp;&emsp; $w_{i}\ =\ x_{1}^{2}+y_{i}^{2}+z_{i}^{2};\ i=1,2,3$


  Solve quadratic equation fing z

  &emsp;&emsp;&emsp; $\left(a_{1}^{2}+a_{2}^{2}+1\right)\cdot z^{2}\ +\ 2\cdot\left(a_{1}+a_{2}\cdot\left(b_{2}-y_{1}\right)-z_{1}\right)\cdot z+\left(b_{1}^{2}+\left(b_{2}-y_{1}\right)^{2}+z_{1}^{2}-r_{e}^{2}\right)\ =\ 0$

  From z find x and y

  &emsp;&emsp;&emsp; $x\ =\ a_{1}z\ +\ b_{1}$

  &emsp;&emsp;&emsp; $y\ =\ a_{2}z\ +\ b_{2}$

  where:

  &emsp;&emsp;&emsp; $a_{1}\ =\ \frac{1}{d}\cdot\left[\left(z_{2}-z_{1}\right)\cdot\left(y_{3}-y_{1}\right)-\left(z_{3}-z_{1}\right)\cdot\left(y_{2}-y_{1}\right)\right]$

  &emsp;&emsp;&emsp; $a_{2}\ =\ -\frac{1}{d}\cdot\left[\left(z_{2}-z_{1}\right)\cdot x_{3}-\left(z_{3}-z_{1}\right)\cdot x_{2}\right]$

  &emsp;&emsp;&emsp; $b_{1}\ =\ -\frac{1}{2d}\cdot\left[\left(w_{2}-w_{1}\right)\cdot\left(y_{3}-y_{1}\right)-\left(w_{3}-w_{1}\right)\cdot\left(y_{2}-y_{1}\right)\right]$

  &emsp;&emsp;&emsp; $b_{2}\ =\ \frac{1}{2d}\cdot\left[\left(w_{2}-w_{1}\right)\cdot x_{3}-\left(w_{3}-w_{1}\right)\cdot x_{2}\right]$

  &emsp;&emsp;&emsp; $d\ =\ \left(y_{2}-y_{1}\right)\cdot x_{3}-\left(y_{3}-y_{1}\right)\cdot x_{2}$

### Inverse Kinematic

<img width="562" alt="delta_robor_simulation" src="https://github.com/ongsa12342/FRA333_DeltaRobot_Simulation/assets/113016544/fe15f623-1c6f-4cdf-b5b6-c6eed5383805">

 &emsp;&emsp;&emsp; $E\left(x_{0},y_{0},z_{0}\right)$

 &emsp;&emsp;&emsp; $EE_{1}=\frac{e}{2}\tan\left(30\right)=\frac{e}{2\sqrt{3}}$

 &emsp;&emsp;&emsp; $E_{1}\left(x_{0},y_{0}-\frac{e}{2\sqrt{3},z_{0}}\right)$

 &emsp;&emsp;&emsp; $E_{1}\left(0,\frac{y_{0}e}{2\sqrt{3}},z_{0}\right)$

 &emsp;&emsp;&emsp; $E_{1}E_{1}'=x_{0}$

 &emsp;&emsp;&emsp; $E_{1}'J_{1}=\sqrt{E_{1}J_{1}^{\ 2}-E_{1}E_{1}'^{2}}=\sqrt{r_{e}^{2}-x_{0}^{2}}$

 &emsp;&emsp;&emsp; $F_{1}\left(0,-\frac{f}{2\sqrt{3}},0\right)$

 &emsp;&emsp;&emsp; $\left(y_{J1}-y_{F1}\right)^{2}+\left(z_{J1}-z_{F1}\right)^{2}=r_{f}^{2}$

 &emsp;&emsp;&emsp; $\left(y_{J1}-y_{F1}\right)^{2}+\left(z_{J1}-z_{F'1}\right)^{2}=r_{e}^{2}-x_{0}^{2}$

 &emsp;&emsp;&emsp; $\left(y_{J1}+\frac{f}{2\sqrt{3}}\right)^{2}+z_{J1}^{2}=r_{f}^{2}$

 &emsp;&emsp;&emsp; $\left(y_{J1}-y_{0}+\frac{e}{2\sqrt{3}}\right)^{2}+\left(z_{J1}-z_{0}\right)^{2}=r_{e}^{2}-x_{0}^{2}$

 &emsp;&emsp;&emsp; $J_{1}\left(0,\ y_{J1},\ z_{J1}\right)$

 &emsp;&emsp;&emsp; $\theta_{1}\ =\ \arctan\left(\frac{z_{J1}}{y_{F1}+y_{J1}}\right)$


### Jacobian Matrix

loop closure equation

&emsp;&emsp;&emsp; $(\overrightarrow{p}+\overrightarrow{r})= \overrightarrow{\textbf{R}}+\overrightarrow{a{i}}+\overrightarrow{b{i}}$

linear velocity

&emsp;&emsp;&emsp; $\hat{b{i}}\cdot \overrightarrow{V} = J{ix} V{x} + J{iy}V{y} + J{iz}V{z};\ i=1,2,3$

where:

&emsp;&emsp;&emsp; $J{ix}=-sin\theta_{3i}cos(\theta_{2i}+\theta_{1i})sin\phi_{i}+cos\theta_{3i}cos\phi_{i}$ 


&emsp;&emsp;&emsp; $J{iy}= sin\theta_{3i}cos(\theta_{2i}+\theta_{1i})cos\phi_{i}+cos\theta_{3i}sin\phi_{i}$

&emsp;&emsp;&emsp; $J{iz}=-sin\theta_{3i}sin(\theta_{2i}+\theta_{1i})$

angular velocity

&emsp;&emsp;&emsp; $\hat{b{i}}\cdot(\overrightarrow{\omega {ai}}\times \overrightarrow{a{i}})=-a\sin\theta_{2i}\sin\theta_{3i}\dot{\theta_{1i}};\ i=1,2,3$

from loop closure equation

&emsp;&emsp;&emsp; $J{p}\overrightarrow{V}=J_{\theta}\dot{\overrightarrow{\theta}}$

### Invese Kinematic Velocity

&emsp;&emsp;&emsp; $\dot{{\overrightarrow{\theta }}}=J^{-1}J{p}\overrightarrow{v}=\frac{Adj(J{\theta})}{det(J{\theta})}J_{p}{\overrightarrow{v}}$




 ### Dynamic Model by Lagrange Method

 Lagrangian Equation

 &emsp;&emsp;&emsp; $L\ =\ K\ -\ U$

 where:    

 Kinetic Energy

 &emsp;&emsp;&emsp; $K\ =\ \frac{1}{4}\cdot\left(2\cdot m_{0}+3\cdot m_{2}\right)\cdot\left(\dot{P_{x}}^{2}+\dot{P_{y}}^{2}+\dot{P_{z}}^{2}\right)+\frac{1}{12}\cdot\left(2\cdot m_{1}+3\cdot m_{2}\right)\cdot r_{f}^{2}\cdot\left(\dot{\theta_{1}}^{2}+\dot{\theta_{2}}^{2}+\dot{\theta_{3}}^{2}\right)$

 Potential Energy

 &emsp;&emsp;&emsp; $U\ =\ \frac{1}{2}\cdot\left(2\cdot m_{0}+3\cdot m_{2}\right)\cdot g\cdot\left(P_{2}^{ }\right)-\frac{1}{2}\cdot\left(m_{1}+m_{2}\right)\cdot g\cdot r_{f}^{ }\cdot\left(\sin\left(\theta_{1}^{ }\right)+\sin\left(\theta_{2}^{ }\right)+\sin\left(\theta_{3}^{ }\right)\right)$

Euler-Lagrange equation

 &emsp;&emsp;&emsp; $\frac{d}{dt}\left(\frac{\partial{L}}{\partial{q}}\right)+\frac{\partial{L}}{\partial{q}}\ =\ T$

 where:

 &emsp;&emsp;&emsp; $\frac{d}{dt}\left(\frac{\partial{L}}{\partial{\dot{q}}}\right)\ =\ \frac{1}{6}\cdot\left(2\cdot m_{1}+3\cdot m_{2}\right)\cdot r_{f}^{2}\cdot\left(\ddot{\theta_{1}}^{2}+\ddot{\theta_{2}}^{2}+\ddot{\theta_{3}}^{2}\right)$ 

 &emsp;&emsp;&emsp; $\frac{\partial{L}}{\partial{q}}\ =\frac{1}{2}\cdot\left(m_{1}+m_{2}\right)\cdot r_{f}\cdot g\cdot\left(\cos\left(\theta_{1}\right)+\cos\left(\theta_{2}\right)+\cos\left(\theta_{3}\right)\right)$

Equation of Motion

 &emsp;&emsp;&emsp; $I\left(q\right)\cdot\ddot{\theta}\ +\ G\left(q\right)\ +B\cdot q =\ T$

 ## Validation
 
 Forward Kinematic
 
 Inverse Kinematic
 
 Dynamic
 
 Cascade Control
 
 Trajectory
 
 - MoveJ
 
 - MoveP

## Acknowledgements
This project is part of the coursework for FRA333 Robot Kinematics at the Institute of Field Robotics, King Mongkut’s University of Technology Thonburi. Special thanks to the course instructors for their guidance and support.

Feel free to explore, modify, and extend this project for educational and research purposes.

## References

Forward & Inverse Kinematic

- https://hypertriangle.com/~alex/delta-robot-tutorial/

Jacobian Matrix

- https://robotics.caltech.edu/~jwb/courses/ME115/handouts/DeltaKinematics

Dynamic

- https://ieeexplore.ieee.org/abstract/document/8015814/authors#authors

- https://www.sciencedirect.com/science/article/pii/S0957415822000095
>>>>>>> ccbbce0ca3493fa04f82f4ac59a3ec8b3b1004b8
