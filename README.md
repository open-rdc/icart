icart
=================

## About

icart packages provides robot models regarding i-Cart series, simulation environments and ypspur control bridge on ROS.

![i-Cart mini](http://wiki.ros.org/Robots/icart_mini?action=AttachFile&do=get&target=icart_mini.png)

## install
### for catkin build
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install python3-catkin-tools
catkin build
```

### build
```
cd ~/catkin_ws/srcgit clone -b noetic-devel https://github.com/open-rdc/icart
wstool init
wstool merge orne-box/orne_box_pkgs.install
wstool up
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/catkin_ws
catkin build
```

### set parameters
```
cd ~/catkin_ws/src/icart/icart_mini_setup/scripts
./create_robot_params
./create_udev_rules
```

## Teleoperation with a joystick

```sh
$ roslaunch icart_mini_driver teleop_joy.launch
```

## License

Copyright (c) 2014 - 2015, [Daiki Maekawa](https://github.com/DaikiMaekawa) and Chiba Institute of Technology. (BSD License)

