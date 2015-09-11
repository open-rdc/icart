icart_mini
=================

[![Stories in Progress](https://badge.waffle.io/open-rdc/icart_mini.svg?label=in progress&title=In Progress)](http://waffle.io/open-rdc/icart_mini)

This package provides packages related to navigation of i-Cart mini in Tsukuba Challenge.

[![Throughput Graph](https://graphs.waffle.io/open-rdc/icart_mini/throughput.svg)](https://waffle.io/open-rdc/icart_mini/metrics) 

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

## Usage

### Bring up the real/simulated robot

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

### Teleoperation with a joystick

```sh
$ roslaunch icart_mini_driver teleop_joy.launch
```

### Build map

```sh
$ roslaunch icart_mini_navigation build_map_teleop.launch
```

During building a map, waypoints are recorded by pressing the No.1 button of the joystick.

When you set 2DNavGoal at the goal point on the RViz, waypoints will be saved and then waypoints file stored in icart_mini_navigation/waypoints_cfg/waypoints.yaml is overwritten. So that, the navigation system can be automatically load waypoints configuration.

If you want to save the map, run a map_saver node like the following command.

```sh
$ rosrun map_server map_saver -f filename
```

Please see [the icart_mini page on the ROS Index](http://rosindex.github.io/r/icart_mini/github-open-rdc-icart_mini/) for more info.

## License

Copyright (c) 2014, Robot Design and Control Lab. (BSD License)

