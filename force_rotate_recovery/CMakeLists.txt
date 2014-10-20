cmake_minimum_required(VERSION 2.8.3)
project(force_rotate_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            costmap_2d
            nav_core
            pluginlib
            base_local_planner
        )

find_package(Eigen REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES force_rotate_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(force_rotate_recovery src/force_rotate_recovery.cpp)
target_link_libraries(force_rotate_recovery ${catkin_LIBRARIES})

install(TARGETS force_rotate_recovery
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES force_rotate_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

