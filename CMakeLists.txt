cmake_minimum_required(VERSION 2.8.3)
project(platform_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_message_files(
   FILES
   command.msg
   platform_control.msg
 )  

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 serial 
 std_msgs
 message_runtime
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# add_executable(gg src/test.cpp)
# target_link_libraries(gg ${catkin_LIBRARIES})

add_executable(platform_driver src/main.cpp)
target_link_libraries(platform_driver ${catkin_LIBRARIES})
add_dependencies(platform_driver platform_driver_generate_messages_cpp)