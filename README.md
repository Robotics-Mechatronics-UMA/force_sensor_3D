# 3-axis_force_sensor_ESP32
Repo for the 3-axis force sensor with the ESP32

## Overview

This package contains a 3-axis force sensor for the EE of a 3-DoF manipulator

**Keywords:** example, package, template

Or, add some keywords to the Bitbucket or GitHub repository.

### License

**Author: Victor Rosillo<br />
University of Malaga
Maintainer: Victor Rosillo Suero, vrosillo1110@gmail.com**

<!-- This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed. -->

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) -->

<!-- ![Delta manipulator](images/Delta_Manipulator.jpeg) -->


### Publications

## Installation

### Installation from Packages
ROS version Melodic
    
Use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/Robotics-Mechatronics-UMA/3-axis_force_sensor_ESP32.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Describe the quickest way to run this software, for example:

Run the force_ros_node, which is ready to read the info written by the ESP32.
    rosrun 3-axis_force_sensor force_ros_node.py


## Config files

params.yaml

## Launch files


## Nodes

Node1 admittance_controller_node: an admittance controller for a 6 DoF parallel manipulator.

Node2 dummy_test_noded: a test node that you can launch if you want to test the admittance controller and check the results.

### ADMITTANCE_CONTROLLER

#### Published Topics

* **`/Force`** ([geometry_msgs/Twist])

	Where the reading of the force sensor will be published. Before reading the serial port information



## Bugs & Feature Requests