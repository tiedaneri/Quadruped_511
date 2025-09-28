
project(task_test)

include_directories("../../../third_party/include")

add_executable(task_test.sun task_test.cpp)
target_link_libraries(task_test.sun third_party)
