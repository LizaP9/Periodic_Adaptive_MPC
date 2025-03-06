cmake_minimum_required(VERSION 3.0.2)
project(raisim_unitree_ros_driver)

set(CMAKE_BUILD_TYPE RELEASE)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
  nav_msgs
  std_msgs
  message_generation
  geometry_msgs
  unitree_legged_msgs
  tf
  raisim
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES raisim_unitree_ros_driver_lib
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
)

add_library(raisim_unitree_ros_driver_lib
  src/raisim_unitree_ros_driver.cpp
)

target_link_libraries(raisim_unitree_ros_driver_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})

    target_link_libraries(${app_name} PUBLIC pthread ${catkin_LIBRARIES} raisim_unitree_ros_driver_lib)

    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()

create_executable(raisim_unitree_ros_driver src/raisim_unitree_ros_driver_node.cpp)

SET (PAKAGE_PATH "${PROJECT_SOURCE_DIR}/")
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/path_list.h.cmake ${CMAKE_BINARY_DIR}/path_list.h)

