cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(CMAKE_CXX_FLAGS "-std=c++0x")

# libcan-prius
find_path(LIBCAN_PRIUS_INCLUDE_DIR sensor/PRIUSReader.h
  /usr/include
  PATH_SUFFIXES
  libcan-prius
)
message(Found ${LIBCAN_PRIUS_INCLUDE_DIR})
find_library(LIBCAN_PRIUS_SENSOR_LIBRARY can-prius-sensor
  /usr/lib
)
find_library(LIBCAN_PRIUS_TYPES_LIBRARY can-prius-types
  /usr/lib
)
include_directories(
  ${LIBCAN_PRIUS_INCLUDE_DIR}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/CANPriusNode.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBCAN_PRIUS_SENSOR_LIBRARY}
  ${LIBCAN_PRIUS_TYPES_LIBRARY})
rosbuild_add_executable(can_prius_node src/can_prius_node.cpp)
target_link_libraries(can_prius_node ${PROJECT_NAME}
  ${LIBCAN_PRIUS_SENSOR_LIBRARY})
