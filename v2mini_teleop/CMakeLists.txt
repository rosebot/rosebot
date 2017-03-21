cmake_minimum_required(VERSION 2.8.3)

project(v2mini_teleop)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs sensor_msgs geometry_msgs tf roslib)

catkin_package(
  CATKIN_DEPENDS roscpp tf rospy std_msgs sensor_msgs geometry_msgs roslib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(teleop
	src/teleop.cpp
	src/teleop_controller.cpp)

target_link_libraries(teleop ${catkin_LIBRARIES} ${SDL2_LIBRARIES} SDL2 ${SDL2_image_LIBRARIES} SDL2_image)

add_dependencies(teleop ${catkin_LIBRARIES})

install(TARGETS teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
