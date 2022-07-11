cmake_minimum_required(VERSION 3.0.2)
project(braccio)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    trajectory_msgs
)

# add_message_files(
#   FILES
# )

add_service_files(
  FILES
  creation.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  trajectory_msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES braccio
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)

add_executable(Command_MUX src/Command_MUX.cpp)
target_link_libraries(Command_MUX ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/Robot_Controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
