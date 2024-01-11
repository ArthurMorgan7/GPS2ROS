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
   JWG.msg
   NEG.msg
   HEADING.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(trans trans.cpp)
target_link_libraries(trans ${catkin_LIBRARIES})

add_executable(testforheading testforheading.cpp)
target_link_libraries(testforheading ${catkin_LIBRARIES})

add_executable(testforbestutma testforbestutma.cpp)
target_link_libraries(testforbestutma ${catkin_LIBRARIES})

add_executable(pubjwg pubjwg.cpp)
target_link_libraries(pubjwg ${catkin_LIBRARIES})

add_executable(pubneg pubneg.cpp)
target_link_libraries(pubneg ${catkin_LIBRARIES})

add_executable(pubheading pubheading.cpp)
target_link_libraries(pubheading ${catkin_LIBRARIES})

add_executable(listener listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(pubrtk_sum pubrtk_sum.cpp)
target_link_libraries(pubrtk_sum ${catkin_LIBRARIES})
