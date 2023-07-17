cmake_minimum_required(VERSION 3.24.1)
project(event_preprocessing_study)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17 -O3 -Wall)
add_compile_options(-std=c++17 -g -Wall)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(event_preprocessor_ros
    src/event_preprocessor_ros.cpp
    src/event_preprocessor.cpp
)

target_link_libraries(event_preprocessor_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

cs_install()
cs_export()