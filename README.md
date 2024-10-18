# Cyber Physical Systems 1 | 2024

## Introduction
This project focuses on setting up the Catkin workspace and developing code to control the movement of the **B1 Dobot** and **Jetson**, alongside Python scripts executed on an individual PC.

Additionally, it also incorporates...

## ROS Workspace: `catkin_ws`

The `catkin_ws` workspace is a ROS-based environment created on our individual Jetson devices. Within this workspace, we developed three key packages:

### 1. `b1_pkg`
This package is responsible for controlling the **B1 DashGo** robot, managing its control, odometry, generation of waypoints, traceability and initiator for the arduino service. 

These scripts are launch by a launch file which launches the serial node from the rosserial package, where it waits a service to be called to start calibrating and sending Yaw and Ultrasonic sensor values. Then, the initatior node that called the service on the serial node.

The next nodes run and connect one to each other to generate the path planning for the B1 Dobot and xArm movement, as well as the reading from the Firebase.

### 2. `xarm_ros`
This package contains all necessary dependencies for simulating and controlling the **xArm 5** robot. It was imported from the [xArm Developer repository](https://github.com/xArm-Developer/xarm_ros).

### 3. `reto_xArm`
This custom package includes scripts developed to control the **xArm** robot through ROS topics and nodes. It facilitates the initiation of movement sequences using topic-based communication for efficient and precise control of the robot.


## Sketches

### IMU2_MPU6050

Contains the sketch utilized by the Arduino Uno board which generates a node and service which publishes the Yaw 
twist and the distance for each ultrasensors through Float32 messages. Also I2C readings and calculations for the MPU6050 are done in a function defined before the start and run where at start it initiates a calibration process after a service is called.

### libraries

Contains the libraries utilized on our Arduino Sketch.

## CAD's

Contains all the CAD files generated for the purpose of adapting the multiple grippers and additions to each of the robots.
