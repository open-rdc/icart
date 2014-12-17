icart_mini
=================

This package provides packages related to navigation of i-Cart mini in Tsukuba Challenge.

## Install

Install ROS software (recommended ROS indigo version with Ubuntu 14.04LTS) at http://www.ros.org/wiki/ROS/Installation, please select Ubuntu platform. 

```sh
$ cd CATKIN_WORKSPACE/src
$ git clone https://github.com/open-rdc/icart_mini
$ wstool init
$ wstool merge icart_mini/icart_mini.install
$ wstool up
$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
$ cd ..
$ catkin_make
```

Note that the libypspur is needed. 

## Usage

#### Bring up the real/simulated robot

The following will show the commands needed to bring up either real or simulated robots.

 * Bring up the simulated robot

```sh
$ roslaunch icart_mini_gazebo icart_mini.launch
```

 * Bring up the real robot

```sh
$ ./icart-mini.sh
$ roslaunch icart_mini_driver icart_mini_drive.launch
```

#### Teleoperation with a joystick

```sh
$ roslaunch icart_mini_driver teleop_joy.launch
```

#### Build map

```sh
$ roslaunch icart_mini_navigation build_map_teleop.launch
```

#### Record the waypoints

 * Using the publish point on RViz

```sh
$ roslaunch icart_mini_navigation record_waypoints_viz.launch
```

 * Using the Joystick

```sh
$ roslaunch icart_mini_navigation record_waypoints_joy.launch
```

#### Navigation

 * Waypoint Navigation

```sh
$ roslaunch icart_mini_navigation play_waypoints_nav.launch
```

 * Path Planning and Navigation in static map

```sh
$ roslaunch icart_mini_navigation nav_static_map.launch
```

## Task Management

https://github.com/open-rdc/TsukubaChallenge

## License

Copyright (c) 2014, Robot Design and Control Lab. (BSD License)

See LICENSE for more info.
