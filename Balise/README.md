# ECAM Eurobot 2024 - Team Erasmus

## Prerequisites
* Ubuntu 20.04, might work on 18.04. 
* ROS Noetic
* opencv-python
* numpy
* numpy-ros
## Setup Instructions
### ROS Installation
Follow the ROS Noetic installation guide: [ROS Noetic Installation for Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)  
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
Calibrate your camera following this tutorial: [Monocular Camera Calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).  
Update the camera resolution and D matrix in the Balise/config/camera{index}.config file.

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
Refer to ROS Network Setup : http://wiki.ros.org/ROS/NetworkSetup. You also need to install ROS on the machine that preview the video feed (use a Ubutnu VM and rviz)

## System Architecture

1. **Camera Node**:
   - Function: open the camera on the device
   - Publishes: `/cam1`, which contains the raw video feed of the camera.
1. **Terrain Node**: 
   - Subscribes to: `/cam1` for receiving camera inputs.
   - Publishes: `/terrain`, which contains the straightened terrain image.

2. **Mask Node**: 
   - Subscribes to: `/terrain` for the straightened terrain image.
   - Function: Compares the received image to a reference image.
   - Publishes: `/mask`, which is the mask of obstacles on the terrain.

3. **Astar Node**: 
   - Subscribes to: `/mask` for obstacle data.
   - Function: Calculates the optimal path avoiding obstacles.
   - Publishes: `/path` which outlines the computed path.

4. **TagsPosition Node**: 
   - Subscribes to: `/terrain` for the straightened terrain image.
   - Function: Identifies and locates tags in the image.
   - Publishes: `/tags/pose` and `/tags/id` indicating the position and identification of tags.
5. **PamiPosition Node**:
  - Subscribes to: `/terrain` for the straightened terrain image and .
  - Function: find the Pami color on the terrain
  - Publishes: `/Pami/videoPosition` indicating the visual position of the Pami.
6. **RobotSender Node**: 
   - Subscribes to: `/path` and `/Pami/CombinedPostion`
   - Function: Compute the robot desired angle and motor power and send it via wifi
   - Publishes: `/Pami/localPosition` indicating the recived position from the Pami (encoder + gyrosocpe).
6. **CombinePosition Node**: 
   - Subscribes to: `/Pami/videoPosition` and `/Pami/localPosition`
   - Function: Compute the robot final robot position
   - Publishes: `/Pami/CombinedPostion`. 
