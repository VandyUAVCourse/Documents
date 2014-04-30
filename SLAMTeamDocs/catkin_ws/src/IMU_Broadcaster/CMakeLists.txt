cmake_minimum_required(VERSION 2.8.3)
project(IMU_Broadcaster)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++0x")

add_executable(Broadcast_Wrapper src/Broadcast_Wrapper.cpp src/Serial_Wrapper.cpp)
add_executable(Broadcast_Listener src/Broadcast_Listener.cpp)

add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

## Specify libraries to link a library or executable target against
target_link_libraries(Broadcast_Wrapper
  ${catkin_LIBRARIES}
  serial
)

target_link_libraries(Broadcast_Listener
   ${catkin_LIBRARIES}
)
