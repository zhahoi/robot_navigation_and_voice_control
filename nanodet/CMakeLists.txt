cmake_minimum_required(VERSION 3.0.2)
project(nanodet)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    bbox.msg
    bboxes.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message("OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(OpenCV_DIR "/home/hit/Softwares/opencv-3.4.10/build")
find_package(OpenCV REQUIRED)

set(ncnn_DIR "/home/hit/Softwares/ncnn/build/install/lib/cmake/ncnn")
find_package(ncnn REQUIRED)
if(NOT TARGET ncnn)
    message(WARNING "ncnn NOT FOUND! Please set ncnn_DIR environment variable")
else()
    message("ncnn found")
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    nanodet
    src/nanodet.cpp
    src/nanodet_node.cpp
)

target_link_libraries(
    nanodet
    ${OpenCV_LIBS}
    ncnn
    ${catkin_LIBRARIES}
)
