if ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
# pass
endif ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
find_path(YPSPUR_INCLUDE_DIR ypspur.h
HINTS /usr/local/include/ /usr/include/ /opt/local/include)
find_path(YPSPUR_LIB_DIR ypspur
HINTS /usr/local/lib /usr/lib /opt/local/lib)
find_library(YPSPUR_LIBRARY ypspur
HINTS /usr/local/lib /usr/lib /opt/local/lib)

find_package_handle_standard_args(YPSpur DEFAULT_MSG YPSPUR_LIBRARY YPSPUR_INCLUDE_DIR)

