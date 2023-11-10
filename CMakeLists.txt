cmake_minimum_required(VERSION 2.8.3)
project(serialPort)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)


add_message_files(
   FILES
   JWG.msg
   NEG.msg
 )
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
add_dependencies(test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pubjwg pubjwg.cpp)
target_link_libraries(pubjwg ${catkin_LIBRARIES})
add_dependencies(pubjwg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pubneg pubneg.cpp)
target_link_libraries(pubneg ${catkin_LIBRARIES})
add_dependencies(pubneg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(listener listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

