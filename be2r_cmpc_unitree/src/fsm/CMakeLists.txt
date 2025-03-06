include_directories("../third-party")
include_directories(SYSTEM "../third-party/qpOASES/include")
include_directories("../third-party/ParamHandler")
include_directories("fsm_states")
include_directories("../controllers/convexMPC")
include_directories("../controllers")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

message ("OPENCV INCLUDES : " ${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE sources "fsm_states/*.cpp" "*.cpp")

add_library(fsm_lib ${sources})
target_link_libraries(fsm_lib
 WBC_Ctrl
 common_lib
 qpOASES
 controllers_lib
 ${catkin_LIBRARIES}
${OpenCV_LIBS}
)
