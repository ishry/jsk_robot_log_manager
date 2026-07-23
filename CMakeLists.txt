cmake_minimum_required(VERSION 3.0.2)
project(jsk_robot_log_manager)

find_package(catkin REQUIRED COMPONENTS
  ak_neuro_vm
  hrpsys_ros_bridge
  message_generation
  rospy
  std_msgs
  std_srvs
)

add_service_files(
  FILES
  StartLogging.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES JskNeuroLogPlugin
  CATKIN_DEPENDS
    ak_neuro_vm
    hrpsys_ros_bridge
    message_runtime
    rospy
    std_msgs
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(JskNeuroLogPlugin SHARED
  src/JskNeuroLogPlugin.cpp
)
target_link_libraries(JskNeuroLogPlugin
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/jsk-robot-log
  scripts/jsk_robot_log_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS JskNeuroLogPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
