cmake_minimum_required(VERSION 2.8.3)
project(icart_mini_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    hardware_interface
    controller_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icart_mini_driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#include_directories(/usr/share/include)

include_directories(${catkin_INCLUDE_DIRS})

#TODO: Support Windows YPSpur
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
find_package(YPSpur)
if(${YPSPUR_FOUND})
# pass
else(${YPSPUR_FOUND})
    MESSAGE(STATUS "YPSpur Not Found")
    MESSAGE(STATUS "Install YPSpur...")
    execute_process(COMMAND sh -c "${CMAKE_CURRENT_SOURCE_DIR}/install_ypspur.sh")
    set(YPSPUR_INCLUDE_DIR /usr/local/include/ypspur.h)
    set(YPSPUR_LIB_DIR /usr/local/lib)
    set(YPSPUR_LIBRARY /usr/local/lib/libypspur.so)
    find_package_handle_standard_args(YPSpur DEFAULT_MSG YPSPUR_LIBRARY YPSPUR_INCLUDE_DIR)
endif(${YPSPUR_FOUND})

add_executable(icart_mini_driver_node src/icart_mini_driver_node.cpp)

target_link_libraries(icart_mini_driver_node
    ${catkin_LIBRARIES}
    ${YPSPUR_LIBRARY}
)

