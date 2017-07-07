cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  node.msg
  nodeArray.msg
)

 add_service_files(
   FILES
   AddTwoInts.srv
   PRM.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
CATKIN_DEPENDS message_runtime
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/SDL2
  /home/nathan/catkin_ws/src/beginner_tutorials/src
)

link_directories(
  /usr/local/lib
  /home/nathan/catkin_ws/src/beginner_tutorials/src
)

add_library(MainGame src/MainGame.cpp)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(HelloWorld src/HelloWorld.cpp)
target_link_libraries(HelloWorld ${catkin_LIBRARIES})
add_dependencies(HelloWorld beginner_tutorials_generate_messages_cpp)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} MainGame SDL2)
add_dependencies(main beginner_tutorials_generate_messages_cpp)

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
