cmake_minimum_required(VERSION 3.0.2)
project(diff_2dnav)

find_package(catkin REQUIRED COMPONENTS
  move_base
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rviz_click_to_2d src/rviz_click_to_2d.cpp)
target_link_libraries(rviz_click_to_2d ${catkin_LIBRARIES})
add_executable(ekf_odom_pub src/ekf_odom_pub.cpp)
target_link_libraries(ekf_odom_pub ${catkin_LIBRARIES})

