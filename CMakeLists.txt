cmake_minimum_required(VERSION 2.8.3)
project(avidbots_cartographer_mapping)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Tell gtest to use c++11 tuple, rather than its own
# This is required for complilation on 16.04 and works in 14.04
add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

set(THIS_PACKAGE_ROSDEPS
  roscpp
  nav_msgs
)
find_package(catkin REQUIRED COMPONENTS
  roslint
  ${THIS_PACKAGE_ROSDEPS}
)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROSDEPS}
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(avidbots_cartographer_offline_mapping_node
  src/cartographer_mapping_base.cpp
  src/cartographer_offline_mapping_node.cpp
  src/cartographer_offline_mapping.cpp
)

add_dependencies(avidbots_cartographer_offline_mapping_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(avidbots_cartographer_offline_mapping_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY
    launch
    param
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


########
## Qt ##
########

include(../qt.cmake)

