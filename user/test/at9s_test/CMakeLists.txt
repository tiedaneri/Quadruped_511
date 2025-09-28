
project(at96s_test)

include_directories("../../../robot/include")

add_executable(at9s_test at9s_test.cpp)
target_link_libraries(at9s_test robot)
