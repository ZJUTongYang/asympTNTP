cmake_minimum_required(VERSION 3.10)
project(asympTNTP)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
  moveit_planners_ompl
)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message (STATUS "CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

## We tell then system we have a catkin_package, no dependencies going forward
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

###############
## Libraries ##
###############
## Declare C++ libraries here
add_library(${PROJECT_NAME}_my_library src/ur5e_kinematics.cpp)

###############
## Executable ##
###############
## Declare C++ executable
add_executable(${PROJECT_NAME} src/main.cpp 
    src/ur5e_kinematics.cpp 
    src/ur5e_controller.cpp
    src/input_data.cpp
    src/environmental_obstacles.cpp
    src/prmstar.cpp
    src/asymp_tntp_algorithm.cpp
    src/ral22_min_reconfig_tntp.cpp)

## Add cmake target dependencies of the executable, enforces generating messages
## VERY VERY VERY HIGHLY unlikely to change
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PROJECT_NAME}_my_library)

#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries