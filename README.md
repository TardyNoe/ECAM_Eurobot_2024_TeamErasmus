# ECAM Eurobot 2024 - Team Erasmus
## Introduction
This repository contains the setup and configuration for the ECAM Eurobot 2024 project by Team Erasmus. It includes steps for ROS installation, workspace configuration, package installation, and camera calibration specifically tailored for the 2024 Eurobot challenge.
## Prerequisites
* Ubuntu 20.04
* ROS Noetic
## Setup Instructions
### ROS Installation
Follow the ROS Noetic installation guide: ROS Noetic Installation for Ubuntu
### Workspace Configuration
Initialize the workspace:
```
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
More information on : http://wiki.ros.org/catkin/Tutorials/create_a_workspace

### Package Installation
Clone the project repository and build the package:
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/TardyNoe/ECAM_Eurobot_2024_TeamErasmus.git
cd ~/catkin_ws/
catkin build
```
  
### Camera Calibration
Calibrate your camera following this tutorial: Monocular Camera Calibration
Update the resolution and D matrix in the camera{index}.config file.
### Configuration
The program is pre-configured for the 2024 Eurobot challenge.
If Aruco tag positions change in future challenges, update the image in Balise/config/RefImage.png.

### Launch Instructions
Launch the core in one terminal:
```
roscore
```
In another terminal, launch the script:
```
source ~/catkin_ws/devel/setup.bash
roslaunch Balise example.launch
```
		  
### Additional Information for Raspberry Pi Users
If using a Raspberry Pi, you can preview the video feed over a local network (note: this may slow down the Pi and preview a slow frame rate). 
Refer to ROS Network Setup : http://wiki.ros.org/ROS/NetworkSetup

Published Topics
* Aruco Tags Position
* Straighten Terrain Image
* Obstacles Mask
* Path
