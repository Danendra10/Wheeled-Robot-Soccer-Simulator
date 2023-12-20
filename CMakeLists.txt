cmake_minimum_required(VERSION 3.0.2)
project(simulator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  utils
  std_msgs
  iris_msgs
)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

message("-- ======== OpenCV_VERSION: ${OpenCV_VERSION} ========")
message("-- ======== OpenCV_LIBS: ${OpenCV_LIBS} ========")
message("-- ======== OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS} ========")
message("-- ======== OpenCV_LIB_DIRS: ${OpenCV_LIB_DIRS} ========")


# add_message_files(
#   FILES
#   robotdata.msg
#   robot2sim.msg
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iris_gazebo
#  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(ball_gazebo src/ball_gazebo.cpp)
add_dependencies(ball_gazebo ${simulation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(robot_gazebo src/robot_gazebo.cpp)
# add_dependencies(robot_gazebo ${simulation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(robot_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} logger_lib)

# add_library(robot_gazebo_new_control src/robot_gazebo_new_control.cpp)
# add_dependencies(robot_gazebo_new_control ${simulation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(robot_gazebo_new_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} logger_lib)

add_library(robot_gazebo_second src/robot_gazebo_second.cpp)
add_dependencies(robot_gazebo_second ${simulation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_gazebo_second ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} logger_lib)

add_library(referee src/referee.cpp)
add_dependencies(referee ${simulation_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(referee ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} logger_lib)
