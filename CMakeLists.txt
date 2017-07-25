cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

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
  testSer.msg
)

 add_service_files(
   FILES
   AddTwoInts.srv
   PRM.srv
   PRMQuery.srv
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
  /home/nathan/catkin_ws/src/beginner_tutorials/src
)

link_directories(
  /usr/local/lib
  /home/nathan/catkin_ws/src/beginner_tutorials/src
)

add_library(MainGame src/MainGame.cpp)
add_library(pQuery src/pQuery.cpp)
add_dependencies(pQuery beginner_tutorials_generate_messages_cpp)
add_dependencies(MainGame beginner_tutorials_generate_messages_cpp)
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(HelloWorld src/HelloWorld.cpp)
target_link_libraries(HelloWorld ${catkin_LIBRARIES})
add_dependencies(HelloWorld beginner_tutorials_generate_messages_cpp)

add_executable(pubPVA src/pubPVA.cpp)
target_link_libraries(pubPVA ${catkin_LIBRARIES})
add_dependencies(pubPVA beginner_tutorials_gencpp)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} MainGame SDL2)
add_dependencies(main beginner_tutorials_generate_messages_cpp)

add_executable(pQueryRun src/pQueryRun.cpp)
target_link_libraries(pQueryRun ${catkin_LIBRARIES} pQuery SDL2)
add_dependencies(pQueryRun beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

add_executable(prmClient src/prmClient.cpp)
target_link_libraries(prmClient ${catkin_LIBRARIES})
add_dependencies(prmClient beginner_tutorials_gencpp)

add_executable(AirSimPRM src/AirSimPRM.cpp)
target_link_libraries(AirSimPRM ${catkin_LIBRARIES})
add_dependencies(AirSimPRM beginner_tutorials_gencpp)

add_executable(moveQuad src/moveQuad.cpp)
target_link_libraries(moveQuad ${catkin_LIBRARIES})
add_dependencies(moveQuad beginner_tutorials_gencpp)
