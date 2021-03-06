cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PIDController.cfg
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pid_controller
  CATKIN_DEPENDS dynamic_reconfigure roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_controller src/pid_controller.cpp src/pid_controller_server.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller pid_controller_gencfg)