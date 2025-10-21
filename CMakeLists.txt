cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release) # 以Release模式构建，如果不设置，程序中矩阵运算极慢导致程序无法正常运行

# 1号机做从机的话，请同时打开两个开关。
add_definitions(-DROBOT1) #机器人1使用
add_definitions(-DFROBOT1) #机器人1做从机使用
# add_definitions(-DROBOT2) #机器人2使用

# 选择控制器
add_definitions(-DONLYMPC)
# add_definitions(-DWBCANDMPC)

add_subdirectory(third_party)
add_subdirectory(robot)
add_subdirectory(user)

