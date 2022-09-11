# robomaster_ros
ROS1 driver for the Robomaster S1 and Robomaster EP through the official SDK. This driver was developed on ROS Noetic.

# Preparation

## Robomaster EP

Connect the robot to the same WiFi as your PC. As far as I know, the SDK should be enabled by default.

## Robomaster S1

On the S1, you need to have the device rooted. This is a fairly simple process for which you can find the tutorial on [this Reddit post](https://www.reddit.com/r/RobomasterS1/comments/lwx45c/robomaster_s1_sdk_hack/). Just follow the instructions in the `README` file in the provided folder.

# Installation

Install catkin tools by executin

    sudo apt install python3-catkin-tools
   
Then, create a catkin workspace and clone this repo into it, together with its dependency catkin_simple

    mkdir catkin_ws && cd catkin_ws && mkdir src
    catkin init
    git clone https://github.com/catkin/catkin_simple.git src/catkin_simple
    git clone https://github.com/jukindle/robomaster_ros.git src/robomaster_ros

Finally, build the workspace

    catkin build
    source devel/setup.bash

# Usage

As soon as the robot and the PC are connected to the same WiFi, you should be able to run the driver by
    
    roslaunch robomaster_driver robomaster_driver.launch
    
In a second terminal, you can launch the teleoperation and visualizazion

    roslaunch robomaster_driver teleop.launch

This will allow you to control the robot with an attached XBOX controller and see the state in rviz (together with the camera image!). Hold `LB` and control the linear velocities with the left and the angular velocity with the right stick.
