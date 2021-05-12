cmake_minimum_required(VERSION 3.14)
project(image)

enable_language(C)
enable_language(CXX)
enable_language(CUDA)
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
cuda_select_nvcc_arch_flags(NVCC_FLAGS "Auto")
string (REPLACE ";" " " CUDA_ARCH_FLAGS_SPACE_SEPARATED "${NVCC_FLAGS}")
set(CMAKE_CUDA_FLAGS "${CUDA_ARCH_FLAGS_SPACE_SEPARATED}")
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV 3.2)
add_definitions(-DGPU)
add_definitions(-DCUDNN)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common
  darknet
  roscpp
  seuimage
  seumath
  roslib
  config
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
set(CMAKE_AUTOMOC ON)
#find_package(OpenCV 3.2)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES remote
#  CATKIN_DEPENDS common config roscpp rospy seumath
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(image_debuger 
  src/ImageLabel.cpp 
  src/image_debuger.cpp
  src/main.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(image_debuger
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  cuda
  cudart
  darknet
  seuimage
)
