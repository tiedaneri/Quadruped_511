
project(gait_test)

include_directories("../../../robot/include")
include_directories("../../../third_party/include")
include_directories("../../leg_controller/include")

add_executable(gait_test gait_test.cpp ../../leg_controller/source/gait_plan.cpp)

