cmake_minimum_required(VERSION 2.8.3)
project(head_ref_ed_client)

find_package(catkin REQUIRED COMPONENTS
  ed_msgs
  head_ref
  roscpp
  geolib2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(head_ref_ed_client src/head_ref_ed_client.cpp)

target_link_libraries(head_ref_ed_client
  ${catkin_LIBRARIES}
)
