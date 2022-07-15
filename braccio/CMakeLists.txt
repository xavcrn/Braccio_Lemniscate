cmake_minimum_required(VERSION 3.0.2)
project(braccio)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    trajectory_msgs
)

add_message_files(
  FILES
  egg_angles.msg
)

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

include_directories(
#    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
)

catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES RF24
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime trajectory_msgs
#  DEPENDS system_lib
)

add_executable(remote_server src/remote_server.cpp)
target_link_libraries(remote_server ${catkin_LIBRARIES})

add_executable(egg_receive src/egg_receive.cpp)
target_link_libraries(egg_receive ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/receiver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/Robot_Controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/control_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
