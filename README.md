# 3-axis_force_sensor
Repo for the 3-axis force sensor

## Overview

This package contains a 3-axis force sensor for the EE of a 3-DoF manipulator

**Keywords:** ESP32, Force, Onshape

### License

**Author: Victor Rosillo<br />
University of Malaga
Maintainer: Victor Rosillo Suero, vrosillo1110@gmail.com**

<!-- This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed. -->

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) -->

<!-- ![Delta manipulator](images/Delta_Manipulator.jpeg) -->


### Publications

Bachelor's Thesis: Adaptive control of a lightweight three-degree-of-freedom parallel
manipulator

## Prerequisites

Before you begin, ensure you have the following installed:

- **Arduino IDE** (version 1.8.13 or latest)
- **ESP32 Board Support** installed in Arduino IDE


## Installation

### Installation from Packages
ROS version Melodic
    
Use `rosdep`:

	sudo rosdep install --from-paths src

To make arduino libraries work, also install the relative dependencies:
```bash
sudo apt install ros-$ROS_DISTRO-rosserial-arduino ros-$ROS_DISTRO-rosserial
```


### 1. Install ESP32 Board Support

1. Open the Arduino IDE.
2. Navigate to `File > Preferences`.
3. In the **Additional Board Manager URLs** field, add the following URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
4. Click **OK**.
5. Now, go to `Tools > Board > Boards Manager`.
6. In the Boards Manager window, search for **ESP32** and click **Install** on the "ESP32 by Espressif Systems" package.

### 2. Select the ESP32 Board

1. Once the installation is complete, go to `Tools > Board`, and from the list, select **ESP32 Dev Module**.
- This will ensure the correct settings are applied for the ESP32 board.

### 3. Connect the ESP32 Board

1. Connect your ESP32 board to your computer using a USB cable.
2. Go to `Tools > Port` and select the correct serial port where your ESP32 is connected.
- If you're unsure, disconnect and reconnect the board to see which port appears or disappears.

### 4. Uploading the Code

1. Open the Arduino sketch you want to upload (e.g., `main.ino`).
2. Verify your code by clicking the checkmark button (**Verify**).
- This will check for any compilation errors.
3. Once verified, click the arrow button (**Upload**) to upload the code to your ESP32.
- During the upload process, you may need to hold the **BOOT** button on the ESP32 board to enter flashing mode.

### 5. Serial Monitor

1. After uploading the code, you can view the output of your ESP32 by opening the **Serial Monitor**.
2. Go to `Tools > Serial Monitor`, or use the shortcut `Ctrl + Shift + M`.
3. Ensure the baud rate matches the one used in your code (typically 115200).

## Troubleshooting

### Common Issues:

1. **Port Not Detected:**
- Ensure the drivers for the ESP32 are correctly installed.
- Try using a different USB cable or port on your computer.

2. **Upload Fails:**
- Hold the **BOOT** button while uploading the code.
- Ensure the correct board and port are selected in the `Tools` menu.

3. **No Output in Serial Monitor:**
- Double-check the baud rate setting in the Serial Monitor window.
- Ensure that the correct COM port is selected.



## Code Structure

The main file for this project is `Force_sensor_node.ino`.


### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/Robotics-Mechatronics-UMA/3-axis_force_sensor_ESP32.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage



## Launch files


## Nodes
* **`force_sensor_node`**


## Topics

### Published Topics

* **`/Force`** ([geometry_msgs/Vector3])

	Where the reading of the force sensor will be published. Before reading the serial port information

### Subscribed Topics

* **`/Tare`** ([geometry_msgs/Vector3])

	Where bias 

## Bugs & Feature Requests
