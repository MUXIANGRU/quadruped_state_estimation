cmake_minimum_required(VERSION 2.8.3)
project(forward_kinematics)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forward_kinematics
  CATKIN_DEPENDS   urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/treefksolverposfull_recursive.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})