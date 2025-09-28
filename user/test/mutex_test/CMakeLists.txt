
project(mutex_test)

# include_directories("./")

add_executable(mutex_test mutex_test.cpp)

target_link_libraries(mutex_test pthread)