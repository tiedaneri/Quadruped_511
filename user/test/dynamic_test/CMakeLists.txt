
project(dynamic_test)

include_directories("../../../robot/include")
include_directories("../../../third_party/include")

add_executable(dynamic_test dynamic_test.cpp)
target_link_libraries(dynamic_test robot third_party)
