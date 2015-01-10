icart_mini
=================

This package provides packages related to navigation of i-Cart mini in Tsukuba Challenge.

[![Stories in Ready](https://badge.waffle.io/open-rdc/icart_mini.svg?label=ready&title=Ready)](http://waffle.io/open-rdc/icart_mini)

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

Please see [the icart_mini page on the ROS wiki](http://wiki.ros.org/icart_mini) for documentation.

## Task Management

https://github.com/open-rdc/TsukubaChallenge

## License

Copyright (c) 2014, Robot Design and Control Lab. (BSD License)

See LICENSE for more info.

