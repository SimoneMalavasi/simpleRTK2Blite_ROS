# simpleRTK2BLite_ROS setup

## Table of contents
* [General info](#general-info)
* [Packages](#packages)
* [Setup](#setup)
* [Launch](#launch)
* [Topics](#topics)

## General info
This project configures a simpleRTK2BLite with a Wi-Fi NTRIP fix on ROS.
	
## Packages
* `ardusimple_rover`: this is the main package to connect simpleRTK2Blite with ROS (https://github.com/msadowski/ardusimple_rover)
* `ntrip_ros`: this package let the NTRIP fix working, once it is correctly configured on EXP-32 XBee Wi-Fi Module (https://github.com/tilk/ntrip_ros)
* `ublox`: it provides support for u-blox GPS receivers (https://github.com/msadowski/ublox)
* `rtcm_msgs`: it contains messages related to data in the RTCM format

## Setup
To run this project, clone this repo inside your ros workspace and compile:

```
$ cd ~/ros_ws/src
$ git clone https://github.com/SimoneMalavasi/simpleRTK2Blite_ROS
$ catkin_make
```

## Launch
Launch the following nodes in order to retrieve data from simpleRTK2Blite and to visualize them on RVIZ

```
$ roslaunch ardusimple_rover ardusimple_rover_pair.launch  
$ roslaunch ntrip_ros ntrip_ros.launch
$ roslaunch ardusimple_rover mapviz.launch
```

## Topics
The most relevant topics are the following ones:
* `/gps1/fix`: it shows gps1 fix data
* `/gps2/fix`: it shows gps1 fix data
* `/diagnostics`: it shows important info about gps connection
* `/rtcm`: it shows rtcm messages coming from NTRIP connection
