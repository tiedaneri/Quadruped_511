cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)

add_definitions(-DROBOT1) #机器人1使用
# add_definitions(-DROBOT2) #机器人2使用

# 选择控制器
add_definitions(-DONLYMPC)
# add_definitions(-DWBCANDMPC)

add_subdirectory(third_party)
add_subdirectory(robot)
add_subdirectory(user)

