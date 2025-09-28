
project(lcm_test)

include_directories("./")

add_executable(lcm_test lcm_test.cpp)
target_link_libraries(lcm_test lcm)
