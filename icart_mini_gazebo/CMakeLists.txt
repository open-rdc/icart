cmake_minimum_required(VERSION 2.8.3)
project(icart_mini_gazebo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    controller_manager
    hardware_interface
    transmission_interface
    pluginlib
    joint_limits_interface
    urdf
    gazebo_ros_control
    #safety_interface
)

find_package(gazebo)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      roscpp
      gazebo_ros
      controller_manager
      pluginlib
      transmission_interface
      gazebo_ros_control
      #safety_interface
    INCLUDE_DIRS
    LIBRARIES gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(icart_mini_hw_sim src/icart_mini_hw_sim.cpp)
target_link_libraries(icart_mini_hw_sim 
    ${catkin_LIBRARIES} 
    ${GAZEBO_LIBRARIES}
)

install(FILES icart_mini_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS icart_mini_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

