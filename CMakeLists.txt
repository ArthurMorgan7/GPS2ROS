cmake_minimum_required(VERSION 2.8.3)
project(gps)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    std_msgs
    novatel_gps_msgs
    message_generation
)
add_message_files( FILES
   NEG.msg
   HEADING.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(trans src/trans.cpp)
target_link_libraries(trans ${catkin_LIBRARIES})

add_executable(pubneg src/pubneg.cpp)
target_link_libraries(pubneg ${catkin_LIBRARIES})

add_executable(pubheading src/pubheading.cpp)
target_link_libraries(pubheading ${catkin_LIBRARIES})

add_executable(pubrtk_sum src/pubrtk_sum.cpp)
target_link_libraries(pubrtk_sum ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

