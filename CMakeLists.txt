cmake_minimum_required(VERSION 2.8.3)
project(prm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  px4_control
  message_generation
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  node.msg
  nodeArray.msg
)

 add_service_files(
   FILES
   PRM.srv
   PRMQuery.srv
   circularMissionPlan.srv
 )

add_action_files(
  DIRECTORY action
  FILES moveQuad.action
)


generate_messages(
   DEPENDENCIES
   px4_control
   std_msgs
   nav_msgs
   actionlib_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime nav_msgs std_msgs actionlib_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/local/include/SDL2
  /home/nathan/catkin_ws/src/prm/src
)

link_directories(
  /usr/local/lib
  /home/nathan/catkin_ws/src/prm/src
)

add_library(populateNodes src/populateNodes.cpp)
add_dependencies(populateNodes prm_generate_messages_cpp)
add_library(pQuery src/pQuery.cpp)
add_dependencies(pQuery prm_generate_messages_cpp)
add_library(circleMP src/circleMP.cpp)
add_dependencies(circleMP prm_generate_messages_cpp)

add_executable(pubPVA src/pubPVA.cpp)
target_link_libraries(pubPVA ${catkin_LIBRARIES})
add_dependencies(pubPVA prm_gencpp)

add_executable(pNodesRun src/pNodesRun.cpp)
target_link_libraries(pNodesRun ${catkin_LIBRARIES} populateNodes SDL2)
add_dependencies(pNodesRun prm_generate_messages_cpp)

add_executable(pQueryRun src/pQueryRun.cpp)
target_link_libraries(pQueryRun ${catkin_LIBRARIES} pQuery SDL2)
add_dependencies(pQueryRun prm_generate_messages_cpp)

add_executable(circleMPRun src/circleMPRun.cpp)
target_link_libraries(circleMPRun ${catkin_LIBRARIES} circleMP)
add_dependencies(circleMPRun prm_generate_messages_cpp)

add_executable(mpClient src/mpClient.cpp)
target_link_libraries(mpClient ${catkin_LIBRARIES} SDL2)
add_dependencies(mpClient prm_generate_messages_cpp)

add_executable(prmClient src/prmClient.cpp)
target_link_libraries(prmClient ${catkin_LIBRARIES})
add_dependencies(prmClient prm_gencpp)

add_executable(AirSimPRM src/AirSimPRM.cpp)
target_link_libraries(AirSimPRM ${catkin_LIBRARIES})
add_dependencies(AirSimPRM prm_gencpp)

add_executable(moveQuad src/moveQuad.cpp)
target_link_libraries(moveQuad ${catkin_LIBRARIES})
add_dependencies(moveQuad prm_gencpp)
