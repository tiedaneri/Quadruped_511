
project(imu_test)

include_directories("../../../third_party/include")
include_directories("../../../third_party/include/lord_imu/Include")

add_executable(imu_test imu_test.cpp)
target_link_libraries(imu_test third_party)
