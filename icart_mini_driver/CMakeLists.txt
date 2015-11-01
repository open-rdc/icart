cmake_minimum_required(VERSION 2.8.3)
project(icart_mini_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    hardware_interface
    controller_manager
    ypspur
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icart_mini_driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${YPSPUR_INCLUDE_DIRS}
)

find_package(YPSpur)

add_executable(icart_mini_driver_node src/icart_mini_driver_node.cpp)

target_link_libraries(icart_mini_driver_node
    ${catkin_LIBRARIES}
    ${YPSPUR_LIBRARY_DIRS}
)

install(FILES
    scripts/ypspur_coordinator_bridge
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

