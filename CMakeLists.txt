cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++17
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-7/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_simulation livox_laser_gpu_simulation
        CATKIN_DEPENDS tf
)

# CPU based laser simulation
add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} RayPlugin)
target_link_libraries(livox_laser_simulation libprotobuf.so)

# GPU accelerated laser simulation
add_library(livox_laser_gpu_simulation SHARED src/livox_gpu_points_plugin.cpp)
target_link_libraries(livox_laser_gpu_simulation ${catkin_LIBRARIES} GpuRayPlugin)

install(TARGETS livox_laser_simulation livox_laser_gpu_simulation
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#target_link_libraries(livox_laser_gpu_simulation libprotobuf.so.9)
#target_compile_definitions(livox_laser_gpu_simulation PRIVATE GAZEBO_GPU_RAY=1)
