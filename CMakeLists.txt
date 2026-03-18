cmake_minimum_required(VERSION 3.0.2)
project(ros1_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SubsonusRemoteTrack.msg
  SubsonusRemoteState.msg
  SubsonusRemoteOrientation.msg  "SubsonusSystemState.msg")

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros1_driver 
  src/an_driver.cpp 
  src/spatial_packets.c 
  src/an_packet_protocol.c 
  src/rs232/rs232.c 
  src/NTRIP_Client/NTRIP/ntripclient.c
  src/subsonus_packets.c
)

add_dependencies(ros1_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros1_driver
   ${catkin_LIBRARIES}
)

install(TARGETS ros1_driver
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
