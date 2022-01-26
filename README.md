# simpleRTK2BLite_ROS setup

## Table of contents
* [General info](#general-info)
* [Packages](#packages)
* [Setup](#setup)
* [Testing Dashboard](#testing-dashboard)
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

## Testing Dashboard
Launch the following launch file in order to use an user-friendly GUI (PySide2 required):

```
$ roslaunch ardusimple_rover dashboard_testing.launch  
```
In the following window is possible to change the default NTRIP Parameters saved in /ntrip_ros/config/ntrip_ros.yaml:

![ntripParams](https://user-images.githubusercontent.com/75474136/151177283-1fc44286-41e6-437c-980f-c74e14015353.png)

* `Server`: ip_address of ntrip server : Port
* `User`: username related to ntrip server
* `Pass`: password related to ntrip server
* `Mountpoint`: mountpoint name of the ntrip server
* `Nmea GGA`: GGA position of the position of GPS testing. It is possible to download it from the link 'NMEA GENERATOR'

In the following window is possible to connect both GPS and to check whether they have FIX (with its percentage) or not and the heading:

![dashboard](https://user-images.githubusercontent.com/75474136/151177362-52392576-96a7-41b5-b91d-63f8355a928c.png)

* `%`: it shows the percentage of FIX RTK over all the GPS sample logged 
* `Connect/Disconnect GPS`: it launches/disconnects the ardusimple_rover_pair.launch file
* `Start/Stop`: it starts/stop the logging of data
* `Bag Name`: the name of the bag where record data
* `Select Bag Folder`: the folder where to save the bag
* `Hz`: it shows the hz of /gps1/fix and /gps2/fix topic
* `Car Image` : it shows the heading

## Launch
Launch the following launch files in order to retrieve data from simpleRTK2Blite and to visualize them on RVIZ

```
$ roslaunch ardusimple_rover ardusimple_rover_pair.launch  
$ roslaunch ntrip_ros ntrip_ros.launch
$ roslaunch ardusimple_rover mapviz.launch
```

## Topics
The most relevant topics are the following ones:
* `/gps1/fix`: it shows gps1 fix data
* `/gps2/fix`: it shows gps2 fix data
* `/diagnostics`: it shows important info about gps connection
* `/rtcm`: it shows rtcm messages coming from NTRIP connection
