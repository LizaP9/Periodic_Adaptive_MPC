cmake_minimum_required(VERSION 3.0.2)
project(be2r_cmpc_unitree)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  unitree_legged_msgs
  geometry_msgs
  tf
  tf2_ros
  nav_msgs
  visualization_msgs
  dynamic_reconfigure
  grid_map_msgs
  grid_map_ros
  cv_bridge
)

# Eigen find
find_package (Eigen3 REQUIRED NO_MODULE PATHS /usr/lib/cmake/eigen3/ NO_DEFAULT_PATH)
set(UNITREE_LEGGED_SDK_PATH "../unitree_legged_sdk")
# set(UNITREE_LEGGED_SDK_PATH "../unitree_legged_sdk-3.4.2")
set(UNITREE_PLATFORM "amd64")

generate_dynamic_reconfigure_options(config/ros_dynamic_params.cfg)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# вынести пути к исходникам отдельных частей mit контроллера в отдельные паременные и уже их кидать в include_directories. Для наглядности

include_directories(
  src
  src/be2r_cmpc_unitree
  src/common
  src/common/FootstepPlanner
  src/common/Controllers
  src/common/ControlParameters
  src/common/Dynamics
  src/common/Math
  src/common/SimUtilities
  src/common/SparseCMPC
  src/common/Utilities
  src/common/debug
  src/controllers
  src/controllers/convexMPC
  src/fsm
  src/fsm/fsm_states
  src/third_party
  src/third_party/ParamHandler
  src/third_party/qpOASES/include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}
  ${UNITREE_LEGGED_SDK_PATH}/include
)

link_directories(${UNITREE_LEGGED_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_${UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME})
add_definitions(-DSDK3_2)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Woverloaded-virtual \
-Wsign-promo")

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
#set(CMAKE_BUILD_TYPE "Debug")

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_BINARY_DIR}/Configuration.h)

add_subdirectory(src/third_party)
add_subdirectory(src/common)
add_subdirectory(src/controllers)
add_subdirectory(src/fsm)

#основная либа ноды
add_library(be2r_cmpc_unitree_lib src/be2r_cmpc_unitree/be2r_cmpc_unitree.cpp)
target_link_libraries(be2r_cmpc_unitree_lib ${catkin_LIBRARIES} controllers_lib fsm_lib Eigen3::Eigen)

#бинарник ноды
add_executable(unitree_ctrl src/be2r_cmpc_unitree_node.cpp)

#зависимости ноды
add_dependencies(unitree_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(unitree_ctrl unitree_legged_msgs_generate_messages_cpp)

#линкуем либы к бинарнику
target_link_libraries(unitree_ctrl ${catkin_LIBRARIES} ${EXTRA_LIBS} be2r_cmpc_unitree_lib Eigen3::Eigen)
add_dependencies(unitree_ctrl ${PROJECT_NAME}_gencfg)
