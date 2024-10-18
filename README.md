# Cyber Physical Systems 1 | 2024

## Introduction
This project centers around setting up the **Catkin workspace** and developing code to control the movement of both the **B1 Dobot** and **Jetson**, alongside Python scripts running on an individual PC. 

Additionally, it integrates various robotic functionalities, such as path planning, sensor data management, and Arduino-based control systems.

## ROS Workspace: `catkin_ws`

The `catkin_ws` workspace is a ROS-based environment set up on individual Jetson devices. Within this workspace, three core packages were developed:

### 1. `b1_pkg`
This package handles control of the **B1 DashGo** robot, managing:
- Control and odometry
- Generation of waypoints for path planning
- Traceability and interaction with Arduino services

The package's operation is initiated by a launch file that starts the serial node from the `rosserial` package. This node waits for a service call to begin calibrating and sending **Yaw** and **Ultrasonic** sensor data. Once this service is called, the initiator node triggers the communication. Subsequent nodes interact to coordinate path planning for both the **B1 Dobot** and **xArm** movements, alongside reading data from Firebase.

### 2. `xarm_ros`
This package contains all required dependencies for simulating and controlling the **xArm 5** robot. It was imported from the [xArm Developer repository](https://github.com/xArm-Developer/xarm_ros) and serves as the foundation for robotic arm control and simulation.

### 3. `reto_xArm`
This custom package includes scripts designed to control the **xArm** robot using ROS topics and nodes. The scripts facilitate movement initiation and coordination through ROS topic-based communication, allowing precise and efficient control of the robot's actions.

## Sketches

### `IMU2_MPU6050`

This sketch, designed for the Arduino Uno board, establishes a node and service that:
- Publishes **Yaw** twist and ultrasonic sensor distance data via `Float32` messages.
- Performs I2C readings and calculations for the **MPU6050** sensor. Calibration is initiated after a service call, and all relevant data is processed and published.

### `libraries`

This folder contains the required libraries for the Arduino sketch, ensuring proper hardware communication and ROS integration.

## CAD Files

This folder contains CAD files designed to adapt multiple grippers and other accessories to both the **B1 Dobot** and **xArm** robots, supporting the customization and functionality of each system.

## [Documentation Report](https://docs.google.com/document/d/1ez1X9worTUOr3sgRrNnTroprc9E3U5eT4iLenxxIJAQ/edit?usp=sharing)

## Team Members

- Carlos Martínez García - A01351950
- Bennet Ávila Bastián - A01275614		
- Jesús Antonio López Malacón - A01742257		              
- Juan Alberto Moreno Cantú - A00833357
- Jorge Eduardo Medina Carvajal  - A00834864                  	
- Hiram David Arguelles Ramirez -A0028630
- Alberto Rojas Casillas - A01275686				
- Carlos Eric González Domínguez - A00834487
- Diego Israel Miranda Valle - A01277295			
- Eliseo Antonio Valdés Castillo - A01412733
- José Fernando Martínez García - A00835291

